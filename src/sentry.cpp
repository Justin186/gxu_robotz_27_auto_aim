#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <optional>
#include <thread>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

// ============================================================================
// 27 赛季哨兵主程序
//
// 结构（三条线程 + 两条由库自己管的后台线程）：
//
//   主线程（相机帧率）        主海康 → yolo → tracker → target_queue
//   Perceptron（库内线程）    后置 USB → yolo → detection_queue
//   控制线程（100Hz）         规划出云台指令 → gimbal.send
//   导航线程（10Hz）          ROS2 cmd_vel/姿态 ⇄ 串口 nav 帧
//   Gimbal（库内线程）        串口收发（视觉帧 + nav 帧）
//
// 几条原则：
//   - io::Gimbal 只是传输层，任何"决策"（扫描、选靶）都不往它里面塞
//   - 下发的是 plan.v_yaw / plan.v_pitch，它们已叠加稳态补偿偏置
//     （plan.yaw / plan.pitch 是纯净目标轨迹，只用于观察）
//   - 全程按内部约定写：pitch 抬头为正，yaw 向左为正。
//     若上机发现 pitch 反向，在 configs/sentry.yaml 里加 pitch_sign: -1，
//     不要改这里的代码
// ============================================================================

using namespace std::chrono_literals;

namespace
{
// 控制线程周期 → 100Hz。云台底层需要比相机帧率更高的指令刷新率
constexpr auto kPlanPeriod = 10ms;

// 导航转发周期 → 10Hz
constexpr auto kNavPeriod = 100ms;

// 导航超过这么久没给新指令就发零停车，避免断连后底盘继续按老指令跑
constexpr double kNavTimeoutSec = 0.3;

// 扫描：匀速扫掠的角速度（rad/s）
constexpr double kScanSpeed = 2.0;

// 丢掉目标后先保持不动这么多帧，给 tracker 一点恢复时间再切扫描
constexpr int kLostFramesBeforeScan = 25;
}  // namespace

const std::string keys =
  "{help h usage ? |                       | 输出命令行参数说明}"
  "{rec            | false                 | 是否录制图像与四元数}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  if (config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  auto record = cli.get<bool>("rec");

  tools::Exiter exiter;
  tools::Recorder recorder;

  // ---- 通信：串口（视觉帧 + nav 帧）与 ROS2（与导航之间）----
  io::Gimbal gimbal(config_path);
  io::ROS2 ros2;

  // ---- 相机：主海康用于自瞄，一路后置 USB 用于扫描找人 ----
  io::Camera camera(config_path);
  io::USBCamera back_camera("video0", config_path);

  // ---- 自瞄 ----
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  // ---- 全向感知：后置相机由 Perceptron 自己在后台跑 ----
  omniperception::Decider decider(config_path);
  omniperception::Perceptron perceptron(&back_camera, config_path);

  // 主线程 → 控制线程。只保留最新一个目标，控制线程不追历史
  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit{false};
  std::atomic<bool> is_detect_enemy{false};

  // ---------------- 控制线程（100Hz）----------------
  auto plan_thread = std::thread([&] {
    bool scanning = false;
    int lost_frames = 0;
    double scan_yaw = 0;
    auto last = std::chrono::steady_clock::now();

    while (!quit) {
      auto now = std::chrono::steady_clock::now();
      double dt = tools::delta_time(now, last);
      last = now;

      auto gs = gimbal.state();
      auto target = target_queue.front();

      if (target.has_value()) {
        // 正常自瞄
        scanning = false;
        lost_frames = 0;

        auto plan = planner.plan(
          target, gs.bullet_speed, gs.yaw, gs.pitch, gs.yaw_offset, gs.pitch_offset);

        gimbal.send(
          plan.control, plan.fire, plan.v_yaw, plan.yaw_vel, plan.yaw_acc, plan.v_pitch,
          plan.pitch_vel, plan.pitch_acc);

      } else {
        // 主相机没目标：用后置相机找人
        auto cmd = decider.decide(perceptron.get_detection_queue());

        if (cmd.control) {
          // 后置相机看到了 —— 转过去，等主相机接上
          scanning = false;
          lost_frames = 0;
          scan_yaw = tools::limit_rad(gs.yaw + cmd.yaw);
          gimbal.send(
            true, false, scan_yaw, 0, 0, tools::limit_rad(cmd.pitch), 0, 0);

        } else if (lost_frames < kLostFramesBeforeScan) {
          // 刚丢目标：原地保持，别急着扫
          ++lost_frames;
          gimbal.send(true, false, gs.yaw, 0, 0, gs.pitch, 0, 0);

        } else {
          // 扫掠：从当前角度开始匀速转，只动 yaw，不动 pitch
          if (!scanning) {
            scanning = true;
            scan_yaw = gs.yaw;
          }
          scan_yaw = tools::limit_rad(scan_yaw + kScanSpeed * dt);
          gimbal.send(true, false, scan_yaw, 0, 0, gs.pitch, 0, 0);
        }
      }

      std::this_thread::sleep_for(kPlanPeriod);
    }
  });

  // ---------------- 导航转发线程（10Hz）----------------
  auto nav_thread = std::thread([&] {
    io::NavCommand last_nav;  // 最近一条非零指令
    auto last_rx = std::chrono::steady_clock::now() - 1s;  // 一开始就当作已超时

    while (!quit) {
      auto now = std::chrono::steady_clock::now();

      // 导航 → 电控
      io::NavCommand nav;
      if (auto cmd = ros2.cmd_vel()) {
        nav.linear_x = cmd->linear.x;
        nav.linear_y = cmd->linear.y;
        nav.linear_z = cmd->linear.z;
        nav.angular_x = cmd->angular.x;
        nav.angular_y = cmd->angular.y;
        nav.angular_z = cmd->angular.z;
        last_nav = nav;
        last_rx = now;
      } else if (std::chrono::duration<double>(now - last_rx).count() < kNavTimeoutSec) {
        // 导航的发布频率和这里的采样频率不一致，超时前沿用上一条，避免一顿一顿
        nav = last_nav;
      }
      // 超时则保持全零 = 停车

      nav.move_mode = ros2.sentry_posture();
      gimbal.send_nav(nav);

      // 电控 → 导航
      ros2.publish_referee(gimbal.nav_state(), is_detect_enemy.load());

      std::this_thread::sleep_for(kNavPeriod);
    }
  });

  // ---------------- 主线程：感知 ----------------
  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);
    if (record) recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    // 给导航的"是否发现敌人"，取 tracker 的结果而不是单帧观测，更稳定
    is_detect_enemy.store(!targets.empty());

    // 发往导航：当前瞄准目标
    ros2.publish_target(decider.get_target_info(armors, targets), !targets.empty());

    if (cv::waitKey(1) == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (nav_thread.joinable()) nav_thread.join();

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  tools::logger()->info("[sentry] 已退出");

  return 0;
}
