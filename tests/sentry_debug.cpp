#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
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
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

// ============================================================================
// 27 赛季哨兵主程序的调试版：逻辑与 src/sentry.cpp 完全一致，
// 额外提供主相机图传：
//   --display   imshow 显示主相机画面（检测框 + 目标重投影）
//
// 结构（三条线程 + 两条由库自己管的后台线程）：
//
//   主线程（相机帧率）        主海康 → yolo → tracker → target_queue
//   Perceptron（库内线程）    左/右 USB → yolo → detection_queue
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

void draw_armors(cv::Mat & img, const std::list<auto_aim::Armor> & armors)
{
  for (const auto & armor : armors) {
    cv::rectangle(img, armor.box, {0, 255, 0}, 1);
    tools::draw_points(img, armor.points, {0, 255, 0});
    tools::draw_text(
      img, fmt::format("{:.2f}", armor.confidence), armor.box.tl(), {0, 255, 0}, 0.6, 1);
  }
}
}  // namespace

const std::string keys =
  "{help h usage ? |                       | 输出命令行参数说明}"
  "{rec            | false                 | 是否录制图像与四元数}"
  "{d display      | false                 | 显示主相机画面（检测框 + 重投影）}"
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
  auto display = cli.get<bool>("display");

  // SSH/无头环境下 imshow 会因 GTK 初始化失败直接 abort，自动关掉
  const bool has_display_env =
    std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
  if (display && !has_display_env) {
    tools::logger()->warn("no DISPLAY/WAYLAND_DISPLAY, --display disabled");
    display = false;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;

  // ---- 通信：串口（视觉帧 + nav 帧）与 ROS2（与导航之间）----
  io::Gimbal gimbal(config_path);
  io::ROS2 ros2;

  // ---- 相机：主海康用于自瞄，左/右 USB 用于侧向寻敌 ----
  io::Camera camera(config_path);
  io::USBCamera left_camera("video0", config_path);
  io::USBCamera right_camera("video2", config_path);

  // ---- 自瞄 ----
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  // ---- 侧向感知：左/右 USB 相机由 Perceptron 自己在后台跑 ----
  omniperception::Decider decider(config_path);
  omniperception::Perceptron perceptron(&left_camera, &right_camera, config_path, display);

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
          // 主相机没目标：用左/右 USB 相机寻找目标
        auto cmd = decider.decide(perceptron.get_detection_queue());

        if (cmd.control) {
          // 侧向相机看到了 —— 转过去，等主相机接上
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

  // 三路相机帧率统计：主相机按主循环计数，左/右 USB 按其采集线程的 capture_count 计数
  uint64_t main_frames = 0;
  uint64_t left_frames_last = left_camera.capture_count();
  uint64_t right_frames_last = right_camera.capture_count();
  auto fps_t0 = std::chrono::steady_clock::now();

  // 最近一次统计到的帧率，供检测到目标时打印
  double main_fps = 0.0, left_fps = 0.0, right_fps = 0.0;
  auto last_target_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  auto last_side_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);

  while (!exiter.exit()) {
    camera.read(img, t);
    ++main_frames;
    auto q = gimbal.q(t);
    if (record) recorder.record(img, q, t);

    // 每秒打印一次三路相机的实际帧率
    auto fps_now = std::chrono::steady_clock::now();
    double fps_elapsed = std::chrono::duration<double>(fps_now - fps_t0).count();
    if (fps_elapsed >= 1.0) {
      uint64_t left_now = left_camera.capture_count();
      uint64_t right_now = right_camera.capture_count();
      main_fps = main_frames / fps_elapsed;
      left_fps = (left_now - left_frames_last) / fps_elapsed;
      right_fps = (right_now - right_frames_last) / fps_elapsed;
      tools::logger()->info(
        "FPS main={:.1f} left={:.1f} right={:.1f}", main_fps, left_fps, right_fps);
      main_frames = 0;
      left_frames_last = left_now;
      right_frames_last = right_now;
      fps_t0 = fps_now;
    }

    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    perceptron.set_pause(tracker.state() != "lost");

    if (!targets.empty()) {
      target_queue.push(targets.front());

      // 检测到目标时打印目标信息 + 三路相机帧率（限频 1Hz，避免刷屏）
      auto tgt_now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(tgt_now - last_target_print).count() >= 1.0) {
        const auto & target = targets.front();
        for (const auto & armor : armors) {
          if (armor.name == target.name) {
            fmt::print(
              "[TARGET] {} {} dist={:.2f}m conf={:.2f} | FPS main={:.1f} left={:.1f} "
              "right={:.1f}\n",
              auto_aim::ARMOR_NAMES[target.name], auto_aim::ARMOR_TYPES[target.armor_type],
              armor.ypd_in_world[2], armor.confidence, main_fps, left_fps, right_fps);
            std::fflush(stdout);
            break;
          }
        }
        last_target_print = tgt_now;
      }
    } else {
      target_queue.push(std::nullopt);
    }

    // 侧向相机检测到目标时打印目标信息 + 三路相机帧率（限频 1Hz）
    auto side_now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(side_now - last_side_print).count() >= 1.0) {
      for (const auto & kv : perceptron.get_latest_armors()) {
        if (!kv.second.empty()) {
          const auto & armor = kv.second.front();
          fmt::print(
            "[SIDE] {}: {} {} conf={:.2f} | FPS main={:.1f} left={:.1f} right={:.1f}\n",
            kv.first, auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type],
            armor.confidence, main_fps, left_fps, right_fps);
          std::fflush(stdout);
          last_side_print = side_now;
          break;
        }
      }
    }

    // 给导航的"是否发现敌人"，取 tracker 的结果而不是单帧观测，更稳定
    is_detect_enemy.store(!targets.empty());

    // 发往导航：当前瞄准目标
    ros2.publish_target(decider.get_target_info(armors, targets), !targets.empty());

    if (display) {
      cv::Mat view = img.clone();
      draw_armors(view, armors);
      // tracker 解算出的目标装甲板位置重投影（黄色），用于验证 solver 是否正确
      for (const auto & target : targets) {
        for (const auto & xyza : target.armor_xyza_list()) {
          auto pts =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(view, pts, {255, 255, 0});
        }
      }
      tools::draw_text(view, tracker.state(), {10, 30}, {255, 255, 255}, 0.8, 1);

      cv::resize(view, view, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
      cv::imshow("sentry_debug", view);

      // 侧向相机图传：直接读采集线程最新帧（与推理解耦，保证实时），检测框异步叠加
      auto side_armors = perceptron.get_latest_armors();
      auto show_side = [&](io::USBCamera & cam) {
        cv::Mat img = cam.get_latest_img();
        if (img.empty()) return;
        draw_armors(img, side_armors[cam.device_name]);
        cv::Mat small;
        cv::resize(img, small, {}, 0.5, 0.5);
        cv::imshow(cam.device_name, small);
      };
      show_side(left_camera);
      show_side(right_camera);

      if (cv::waitKey(1) == 'q') break;
    } else if (has_display_env && cv::waitKey(1) == 'q')
      break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (nav_thread.joinable()) nav_thread.join();

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  tools::logger()->info("[sentry_debug] 已退出");

  return 0;
}
