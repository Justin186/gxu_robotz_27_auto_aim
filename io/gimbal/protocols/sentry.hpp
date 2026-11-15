#ifndef IO__GIMBAL_PROTOCOLS_SENTRY_HPP
#define IO__GIMBAL_PROTOCOLS_SENTRY_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "io/gimbal/frame.hpp"
#include "io/gimbal/protocols/standard.hpp"
#include "io/gimbal/types.hpp"
#include "tools/crc.hpp"

namespace io
{
// ---------------------------------------------------------------------------
// 哨兵协议 = 标准协议的视觉帧 + 一条导航通道，两种帧走同一个串口
//
//   视觉帧与标准协议**完全相同**（直接复用 StandardProtocol<HasOffset>），
//   所以"电控有没有实现稳态补偿偏置"这个差异自动继承过来，帧长同样是 45 / 43。
//
//   导航通道是额外的两组帧：
//     上行 29 字节  {0xB5, 0xA5} | 血量/热量/姿态/比赛状态/位置… | crc16
//     下行 28 字节  0xA5 | 六轴速度 f32 ×6 | move_mode u8 | crc16
//
// 注意 NavTx 的帧头只有**一个**字节，与另外三个双字节帧头不同 —— 这是电控那边
// 既有的约定，照抄不动。
// ---------------------------------------------------------------------------
template <bool HasOffset>
struct SentryProtocol
{
  static constexpr bool has_offset = HasOffset;
  static constexpr bool has_nav = true;

  static constexpr const char * name() { return HasOffset ? "sentry" : "sentry-no-offset"; }

  // 视觉帧复用标准协议的定义
  using Vision = StandardProtocol<HasOffset>;
  using VisionRx = typename Vision::RxFrame;
  using VisionTx = typename Vision::TxFrame;

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

  static bool rx_check(const RxFrameSpec & spec, const uint8_t * frame, tools::CrcByteOrder order)
  {
    return tools::check_crc16(frame, spec.size, order);
  }

  static void rx_decode(const RxFrameSpec & spec, const uint8_t * frame, RxOut & out)
  {
    if (spec.kind == RxKind::Vision) {
      // 视觉帧与标准协议完全一致，直接复用
      Vision::rx_decode(spec, frame, out);
      return;
    }

    NavRx f;
    std::memcpy(&f, frame, sizeof(f));

    auto & nf = out.nav;
    nf.vulnerability_buff = f.vulnerability_buff;
    nf.current_hp = f.current_hp;
    nf.shooter_17mm_barrel_heat = f.shooter_17mm_barrel_heat;
    nf.projectile_allowance_17mm = f.projectile_allowance_17mm;
    nf.current_posture = f.current_posture;
    nf.exchanged_ammo_total = f.exchanged_ammo_total;
    nf.game_progress = f.game_progress;
    nf.stage_remain_time = f.stage_remain_time;
    nf.outpost_Hp = f.outpost_Hp;
    nf.base_Hp = f.base_Hp;
    nf.pos_x = f.pos_x;
    nf.pos_y = f.pos_y;
  }

  static constexpr size_t tx_frame_size = sizeof(VisionTx);

  static void tx_encode(const GimbalCommand & cmd, uint8_t * frame, tools::CrcByteOrder order)
  {
    Vision::tx_encode(cmd, frame, order);
  }

  static constexpr size_t tx_nav_frame_size = sizeof(NavTx);

  static void tx_encode_nav(const NavCommand & cmd, uint8_t * frame, tools::CrcByteOrder order)
  {
    NavTx f{};
    f.linear_x = cmd.linear_x;
    f.linear_y = cmd.linear_y;
    f.linear_z = cmd.linear_z;
    f.angular_x = cmd.angular_x;
    f.angular_y = cmd.angular_y;
    f.angular_z = cmd.angular_z;
    f.move_mode = cmd.move_mode;

    tools::put_crc16(reinterpret_cast<uint8_t *>(&f), sizeof(f) - sizeof(f.crc16), order);

    std::memcpy(frame, &f, sizeof(f));
  }
};

}  // namespace io

#endif  // IO__GIMBAL_PROTOCOLS_SENTRY_HPP
