#ifndef IO__GIMBAL_TYPES_HPP
#define IO__GIMBAL_TYPES_HPP

#include <cstdint>

#include <Eigen/Geometry>

namespace io
{
// ============================================================================
// 内部约定（全项目只需要记住这一条）
//
//   yaw            机器人/云台坐标系下的朝向，rad，向左为正
//   pitch          抬头为正，rad
//   yaw_vel / pitch_vel / yaw_acc / pitch_acc   同向，rad/s、rad/s²
//   yaw_offset / pitch_offset                   电控传来的偏置，单位 0.1 度（i8）
//
// 与电控实际符号不一致的兵种（比如电控把 pitch 的正负搞反了），由 io::Gimbal
// 按 config 里的 pitch_sign 做双向翻译（收、发都翻），**业务代码一律按上面的
// 约定写，不要自己加负号**。
//
// CRC 字节序同理，由 config 里的 crc_byte_order 决定。
// ============================================================================

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

// ---------------------------------------------------------------------------
// 云台反馈 / 控制量
//
// 这两个结构与串口线上的字节布局无关，各协议实现负责把自己的帧翻译成它们。
// 上层代码只认这两个结构，因此协议怎么变都不影响业务逻辑。
// ---------------------------------------------------------------------------
struct GimbalFeedback
{
  Eigen::Quaterniond q{1, 0, 0, 0};  // wxyz 顺序
  float yaw = 0;
  float yaw_vel = 0;
  float pitch = 0;  // 抬头为正（若电控相反，由 Gimbal 按 pitch_sign 翻转）
  float pitch_vel = 0;
  float bullet_speed = 0;
  uint16_t bullet_count = 0;
  int8_t yaw_offset = 0;    // 0.1 度
  int8_t pitch_offset = 0;  // 0.1 度
  GimbalMode mode = GimbalMode::IDLE;
  uint8_t raw_mode = 0;  // 线上的原始 mode，仅用于日志
  bool mode_valid = true;
};

struct GimbalCommand
{
  bool control = false;
  bool fire = false;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;  // 抬头为正（若电控相反，由 Gimbal 按 pitch_sign 翻转）
  float pitch_vel = 0;
  float pitch_acc = 0;
};

// ---------------------------------------------------------------------------
// 导航通道（只有哨兵那条协议有这个通道，走同一个串口、另一种帧头）
// ---------------------------------------------------------------------------
struct NavCommand
{
  float linear_x = 0;
  float linear_y = 0;
  float linear_z = 0;
  float angular_x = 0;
  float angular_y = 0;
  float angular_z = 0;
  uint8_t move_mode = 0;  // 姿态：1=进攻 2=防御 3=移动
};

struct NavFeedback
{
  uint8_t vulnerability_buff = 0;          // 机器人负防御增益
  uint16_t current_hp = 0;                 // 当前血量
  uint16_t shooter_17mm_barrel_heat = 0;   // 枪口热量
  uint16_t projectile_allowance_17mm = 0;  // 剩余允许发弹量
  uint8_t current_posture = 0;             // 当前姿态
  uint16_t exchanged_ammo_total = 0;       // 累计成功兑换的允许发弹量
  uint8_t game_progress = 0;               // 当前比赛阶段
  uint16_t stage_remain_time = 0;          // 当前阶段剩余时间
  uint16_t outpost_Hp = 0;                 // 前哨站血量
  uint16_t base_Hp = 0;                    // 基地血量
  float pos_x = 0;                         // 位置 x (米)
  float pos_y = 0;                         // 位置 y (米)
};

// 一次上行解析的输出。按帧的 RxKind 取对应那一半，另一半保持默认值。
struct RxOut
{
  GimbalFeedback vision;
  NavFeedback nav;
};

}  // namespace io

#endif  // IO__GIMBAL_TYPES_HPP
