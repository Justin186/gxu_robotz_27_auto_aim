#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>

#include <Eigen/Geometry>

#include "io/gimbal/frame.hpp"
#include "io/gimbal/protocol.hpp"
#include "io/gimbal/types.hpp"
#include "serial/serial.h"
#include "tools/crc.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
// 云台状态快照（与协议无关）
struct GimbalState
{
  float yaw = 0;
  float yaw_vel = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float bullet_speed = 0;
  uint16_t bullet_count = 0;
  int8_t yaw_offset = 0;
  int8_t pitch_offset = 0;
};

// 云台串口：负责收发帧、维护状态、给出插值后的四元数。
//
// 具体线上格式由编译期选定的 GimbalProtocol 决定：
//   - 读线程按协议声明的帧表（rx_frames）按帧头分派，所以同一条串口上可以有
//     多种帧（哨兵就是「视觉帧 + 导航帧」两种）
//   - 本类不关心字节长什么样，只做「找帧 → 校验 → 解析 → 存状态」
//
// 与电控的"约定差异"（CRC 字节序等）由配置决定，集中在构造函数里读取并打印，
// 不散落到业务代码。校验配错时会打一条日志告诉你该改成什么。
//
// 导航通道（nav_state / send_nav）只有在协议声明 has_nav 时才可用：
// 用标准协议的兵种去调用会直接编译报错，而不是静默无效。
class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  // ---------------- 视觉通道 ----------------

  void send(const GimbalCommand & cmd);
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  // ---------------- 导航通道（仅哨兵）----------------

  // 最近一帧导航状态（血量 / 热量 / 姿态 / 比赛状态 / 位置…）
  template <
    typename P = GimbalProtocol, typename = std::enable_if_t<P::has_nav>>
  NavFeedback nav_state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return nav_state_;
  }

  // 下发底盘速度指令
  template <
    typename P = GimbalProtocol, typename = std::enable_if_t<P::has_nav>>
  void send_nav(const NavCommand & cmd)
  {
    uint8_t frame[P::max_frame_size];
    P::tx_encode_nav(cmd, frame, crc_order_);
    write_frame(frame, P::tx_nav_frame_size);
  }

  // 当前使用的 CRC 字节序（诊断日志用）
  tools::CrcByteOrder crc_byte_order() const { return crc_order_; }

protected:
  // 放在 protected：将来若有子类也要往同一个串口写帧，必须共用同一把锁
  mutable std::mutex serial_mutex_;
  serial::Serial serial_;

private:
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  NavFeedback nav_state_;

  // 不同电控的 CRC 字节序可能不一样，由 config 里的 crc_byte_order 指定
  tools::CrcByteOrder crc_order_ = tools::CrcByteOrder::Little;
  // "字节序配错了"的提示只打一次，避免刷屏
  std::atomic<bool> crc_hint_printed_{false};

  // 有的电控把 pitch 的正负搞反了，由 config 里的 pitch_sign 指定（1 或 -1）。
  // 收发两个方向各翻一次，业务代码一律按"抬头为正"写，不再手写负号。
  float pitch_sign_ = 1.0f;

  // 反馈帧里的 yaw/pitch 稳态补偿偏置要不要用（config 的 use_gimbal_offset）。
  // 注意这只影响"用不用"，不改变帧布局 —— 所以运行期可切、可测。
  // 如果某块电控的帧里压根没有这两个字节，那是另一份协议实现（帧长不同），
  // 不是这个开关能表达的。
  bool use_gimbal_offset_ = true;

  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void write_frame(const uint8_t * frame, size_t size);
  void read_thread();
  void reconnect();

  // 按帧头首字节在协议的帧表里查一帧；查不到返回 nullptr
  static const RxFrameSpec * find_rx_frame(uint8_t head0);
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
