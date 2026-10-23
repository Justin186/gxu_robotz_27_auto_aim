#ifndef IO__GIMBAL_PROTOCOLS_SENTRY_HPP
#define IO__GIMBAL_PROTOCOLS_SENTRY_HPP

#include <cstddef>
#include <cstdint>

#include "io/gimbal/frame.hpp"
#include "io/gimbal/protocols/standard.hpp"
#include "io/gimbal/types.hpp"
#include "tools/crc.hpp"

namespace io
{
// ---------------------------------------------------------------------------
// 哨兵协议 = 标准协议的视觉帧 + 一条导航通道，两种帧走同一个串口
//
//   视觉帧与标准协议**完全相同**（'S' 'P'，45 / 29 字节），直接复用
//   StandardProtocol 的实现 —— 电控那边这两帧本来就是同一套。
//
//   导航通道是额外的两组帧：
//     上行 29 字节  {0xB5, 0xA5} | 血量/热量/姿态/比赛状态/位置… | crc16
//     下行 28 字节  0xA5 | 六轴速度 f32 ×6 | move_mode u8 | crc16
//
// 注意 NavTx 的帧头只有**一个**字节，与另外三个双字节帧头不同 —— 这是电控那边
// 既有的约定，照抄不动。
// ---------------------------------------------------------------------------
struct SentryProtocol
{
  static constexpr const char * name() { return "sentry"; }
  static constexpr bool has_nav = true;

  // 电控在反馈帧里带 yaw/pitch 稳态补偿偏置（沿用标准协议那套视觉帧）
  static constexpr bool has_gimbal_offset = true;

  // 视觉帧复用标准协议的定义
  using VisionRx = StandardProtocol::RxFrame;
  using VisionTx = StandardProtocol::TxFrame;

  struct __attribute__((packed)) NavRx
  {
    uint8_t head[2] = {0xB5, 0xA5};
    uint8_t vulnerability_buff;          // 机器人负防御增益
    uint16_t current_hp;                 // 当前血量
    uint16_t shooter_17mm_barrel_heat;   // 枪口热量
    uint16_t projectile_allowance_17mm;  // 剩余允许发弹量
    uint8_t current_posture;             // 当前姿态 (1=进攻 2=防御 3=移动)
    uint16_t exchanged_ammo_total;       // 累计成功兑换的允许发弹量
    uint8_t game_progress;               // 当前比赛阶段
    uint16_t stage_remain_time;          // 当前阶段剩余时间
    uint16_t outpost_Hp;                 // 前哨站血量
    uint16_t base_Hp;                    // 基地血量
    float pos_x;                         // 位置 x (米)
    float pos_y;                         // 位置 y (米)
    uint16_t crc16;
  };

  struct __attribute__((packed)) NavTx
  {
    uint8_t head = 0xA5;  // 单字节帧头
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
    uint8_t move_mode;  // 姿态：1=进攻 2=防御 3=移动
    uint16_t crc16;
  };

  static_assert(sizeof(NavRx) == 29, "哨兵协议导航上行帧必须是 29 字节");
  static_assert(sizeof(NavTx) == 28, "哨兵协议导航下行帧必须是 28 字节");

  static constexpr size_t max_frame_size = 64;

  static constexpr RxFrameSpec rx_frames[] = {
    {'S', 'P', sizeof(VisionRx), RxKind::Vision},
    {0xB5, 0xA5, sizeof(NavRx), RxKind::Nav},
  };
  static constexpr size_t rx_frame_count = 2;

  static bool rx_check(
    const RxFrameSpec & spec, const uint8_t * frame, tools::CrcByteOrder order);
  static void rx_decode(const RxFrameSpec & spec, const uint8_t * frame, RxOut & out);

  static constexpr size_t tx_frame_size = sizeof(VisionTx);
  static void tx_encode(const GimbalCommand & cmd, uint8_t * frame, tools::CrcByteOrder order);

  static constexpr size_t tx_nav_frame_size = sizeof(NavTx);
  static void tx_encode_nav(
    const NavCommand & cmd, uint8_t * frame, tools::CrcByteOrder order);
};

}  // namespace io

#endif  // IO__GIMBAL_PROTOCOLS_SENTRY_HPP
