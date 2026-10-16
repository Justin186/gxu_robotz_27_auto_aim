#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include <Eigen/Geometry>

#include "io/gimbal/protocol.hpp"
#include "io/gimbal/types.hpp"
#include "serial/serial.h"
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
// 具体线上格式由编译期选定的 GimbalProtocol 决定，本类不关心字节长什么样。
class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  // 下发控制量
  void send(const GimbalCommand & cmd);
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

protected:
  // 放在 protected：子类（如 GimbalNode）也要往同一个串口写 nav 帧，
  // 必须共用同一把锁，否则和 read_thread 的读会互相踩
  mutable std::mutex serial_mutex_;
  serial::Serial serial_;

private:
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
