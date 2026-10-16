#ifndef IO__GIMBAL_TYPES_HPP
#define IO__GIMBAL_TYPES_HPP

#include <cstdint>

#include <Eigen/Geometry>

namespace io
{
enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

// 视觉侧关心的云台反馈，与串口线上的字节布局无关。
// 各协议实现负责把自己的帧解析成这个结构，上层代码因此不随协议变化。
struct GimbalFeedback
{
  Eigen::Quaterniond q{1, 0, 0, 0};  // wxyz 顺序
  float yaw = 0;
  float yaw_vel = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float bullet_speed = 0;
  uint16_t bullet_count = 0;
  int8_t yaw_offset = 0;
  int8_t pitch_offset = 0;
  GimbalMode mode = GimbalMode::IDLE;
  uint8_t raw_mode = 0;  // 线上的原始 mode，仅用于日志
  bool mode_valid = true;
};

// 视觉侧下发给电控的控制量
struct GimbalCommand
{
  bool control = false;
  bool fire = false;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
};

}  // namespace io

#endif  // IO__GIMBAL_TYPES_HPP
