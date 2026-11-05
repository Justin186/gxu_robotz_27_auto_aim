// 云台 + 底盘通信联调测试（哨兵协议）：视觉帧和导航帧两条通道都测。
//
// 视觉帧：60Hz 发送云台 yaw/pitch 与开火，打印电控回传的云台状态。
// 导航帧：10Hz 发送底盘速度与姿态，打印电控回传的血量/热量/位置。
//
// 只在 GIMBAL_PROTOCOL=sentry 时加入构建（标准协议没有导航通道）。
//
// 终端里直接敲命令，回车生效：
//   yaw <deg>      云台 yaw 绝对值（度，正为逆时针）
//   pitch <deg>    云台 pitch 绝对值（度，正为抬头）
//   spin <deg/s>   云台 yaw 持续转速
//   fire <0|1>     开火开关
//   vx <m/s>       底盘前后速度
//   vy <m/s>       底盘左右速度
//   wz <rad/s>     底盘旋转速度（小陀螺）
//   mode <1|2|3>   底盘姿态：1=进攻 2=防御 3=移动
//   stop           云台和底盘全部归零
//   status         立即打印一帧反馈
//   help           显示命令列表
//   quit           退出（Ctrl+C 也行）

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | yaml 配置文件路径 }";

// 底盘速度安全上限，防止测试时顶到人或撞墙
constexpr float kMaxLinear = 0.5f;  // m/s
constexpr float kMaxAngular = 3.0f; // rad/s

// 命令行按"度"输入，协议内部一律弧度
constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
constexpr float kRad2Deg = 180.0f / 3.14159265358979323846f;

struct Command
{
  std::mutex m;
  float yaw = 0;
  float pitch = 0;
  float spin = 0;
  bool fire = false;
  float vx = 0;
  float vy = 0;
  float wz = 0;
  uint8_t move_mode = 3;
} cmd;

void print_help()
{
  std::cout
    << "yaw <deg>      云台 yaw 绝对值（度）\n"
       "pitch <deg>    云台 pitch 绝对值（度）\n"
       "spin <deg/s>   云台 yaw 持续转速\n"
       "fire <0|1>     开火\n"
       "vx <m/s>       底盘前后速度（限幅 ±0.5）\n"
       "vy <m/s>       底盘左右速度（限幅 ±0.5）\n"
       "wz <rad/s>     底盘旋转速度（限幅 ±3.0）\n"
       "mode <1|2|3>   底盘姿态 1=进攻 2=防御 3=移动\n"
       "stop           全部归零\n"
       "status         立即打印反馈\n"
       "quit           退出\n";
}

void print_status(io::Gimbal & gimbal);

// 返回 false 表示要退出
bool handle_line(const std::string & line, io::Gimbal & gimbal)
{
  std::istringstream iss(line);
  std::string op;
  if (!(iss >> op)) return true;

  std::lock_guard<std::mutex> lock(cmd.m);

  if (op == "quit" || op == "q") return false;
  if (op == "help" || op == "h") {
    print_help();
  } else if (op == "yaw") {
    iss >> cmd.yaw;
  } else if (op == "pitch") {
    iss >> cmd.pitch;
  } else if (op == "spin") {
    iss >> cmd.spin;
  } else if (op == "fire") {
    int v = 0;
    iss >> v;
    cmd.fire = v != 0;
  } else if (op == "vx") {
    iss >> cmd.vx;
    cmd.vx = std::clamp(cmd.vx, -kMaxLinear, kMaxLinear);
  } else if (op == "vy") {
    iss >> cmd.vy;
    cmd.vy = std::clamp(cmd.vy, -kMaxLinear, kMaxLinear);
  } else if (op == "wz") {
    iss >> cmd.wz;
    cmd.wz = std::clamp(cmd.wz, -kMaxAngular, kMaxAngular);
  } else if (op == "mode") {
    int v = 3;
    iss >> v;
    cmd.move_mode = static_cast<uint8_t>(std::clamp(v, 1, 3));
  } else if (op == "stop") {
    cmd.yaw = 0;
    cmd.pitch = 0;
    cmd.spin = 0;
    cmd.fire = false;
    cmd.vx = 0;
    cmd.vy = 0;
    cmd.wz = 0;
  } else if (op == "status") {
    print_status(gimbal);
  } else {
    std::cout << "未知命令: " << op << "（输入 help 查看）\n";
  }
  return true;
}

void print_status(io::Gimbal & gimbal)
{
  auto st = gimbal.state();
  tools::logger()->info(
    "[gimbal] yaw={:.1f}deg vyaw={:.1f}deg/s pitch={:.1f}deg vpitch={:.1f}deg/s "
    "bullet_speed={:.1f} count={} mode={}",
    st.yaw * kRad2Deg, st.yaw_vel * kRad2Deg, st.pitch * kRad2Deg, st.pitch_vel * kRad2Deg,
    st.bullet_speed, st.bullet_count, gimbal.str(gimbal.mode()));

  auto nav = gimbal.nav_state();
  tools::logger()->info(
    "[nav] hp={} heat={} ammo={} posture={} progress={} remain={}s pos=({:.2f}, {:.2f})",
    nav.current_hp, nav.shooter_17mm_barrel_heat, nav.projectile_allowance_17mm,
    nav.current_posture, nav.game_progress, nav.stage_remain_time, nav.pos_x, nav.pos_y);
}

}  // namespace

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help")) {
    cli.printMessage();
    print_help();
    return 0;
  }

  tools::Exiter exiter;
  io::Gimbal gimbal(config_path);

  tools::logger()->info(
    "协议: {}（{}偏移），CRC 字节序: {}", io::GimbalProtocol::name(),
    io::GimbalProtocol::has_offset ? "带" : "不带",
    gimbal.crc_byte_order() == tools::CrcByteOrder::Big ? "大端" : "小端");

  print_help();

  // 输入线程：逐行读取命令，EOF 时退出
  std::atomic<bool> quit = false;
  std::thread input_thread([&] {
    std::string line;
    while (!quit && std::getline(std::cin, line)) {
      if (!handle_line(line, gimbal)) {
        quit = true;
        break;
      }
    }
    quit = true;
  });

  auto t_start = std::chrono::steady_clock::now();
  auto t_last_status = t_start;
  auto t_last_nav = t_start;
  auto t_last_loop = t_start;
  auto t_next = t_start;

  while (!exiter.exit() && !quit) {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - t_last_loop).count();
    t_last_loop = now;

    float yaw, pitch, spin;
    bool fire;
    {
      std::lock_guard<std::mutex> lock(cmd.m);
      // spin 让 yaw 持续累加，超过 ±180 度就环绕回来
      cmd.yaw += cmd.spin * dt;
      while (cmd.yaw > 180) cmd.yaw -= 360;
      while (cmd.yaw < -180) cmd.yaw += 360;
      yaw = cmd.yaw;
      pitch = cmd.pitch;
      spin = cmd.spin;
      fire = cmd.fire;
    }

    // 视觉帧：60Hz（命令行给的是度，协议里是弧度）
    gimbal.send(
      true, fire, yaw * kDeg2Rad, spin * kDeg2Rad, 0, pitch * kDeg2Rad, 0, 0);

    // 导航帧：10Hz
    if (now - t_last_nav >= 100ms) {
      t_last_nav = now;
      float vx, vy, wz;
      uint8_t move_mode;
      {
        std::lock_guard<std::mutex> lock(cmd.m);
        vx = cmd.vx;
        vy = cmd.vy;
        wz = cmd.wz;
        move_mode = cmd.move_mode;
      }
      io::NavCommand nav;
      nav.linear_x = vx;
      nav.linear_y = vy;
      nav.linear_z = 0;
      nav.angular_x = 0;
      nav.angular_y = 0;
      nav.angular_z = wz;
      nav.move_mode = move_mode;
      gimbal.send_nav(nav);
    }

    // 状态反馈：1Hz
    if (now - t_last_status >= 1s) {
      t_last_status = now;
      print_status(gimbal);
    }

    t_next += 9ms;
    // 万一某轮被状态打印拖慢，重新对齐，别让循环空转补帧
    if (t_next < now) t_next = now;
    std::this_thread::sleep_until(t_next);
  }

  quit = true;
  if (input_thread.joinable()) input_thread.detach();

  // 退出前把云台锁住、底盘归零，避免车子继续跑
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  gimbal.send_nav(io::NavCommand{});

  tools::logger()->info("已停止发送，退出");
  return 0;
}
