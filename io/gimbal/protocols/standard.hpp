#ifndef IO__GIMBAL_PROTOCOLS_STANDARD_HPP
#define IO__GIMBAL_PROTOCOLS_STANDARD_HPP

#include <cstddef>
#include <cstdint>

#include "io/gimbal/frame.hpp"
#include "io/gimbal/types.hpp"

namespace io
{
// ---------------------------------------------------------------------------
// 26 赛季标准云台协议（重装 / 轮腿步兵 / 狗洞步兵共用）
//
// 线上布局与 26 赛季完全一致（packed + 小端），并用 static_assert 把字节数钉死：
//
//   上行 45 字节
//     'S' 'P' | mode u8 | q[4] f32 | yaw f32 | yaw_vel f32 | pitch f32 |
//     pitch_vel f32 | bullet_speed f32 | bullet_count u16 |
//     yaw_offset i8 | pitch_offset i8 | crc16 u16
//
//   下行 29 字节
//     'S' 'P' | mode u8 | yaw/yaw_vel/yaw_acc f32 |
//     pitch/pitch_vel/pitch_acc f32 | crc16 u16
//
// CRC16 覆盖整帧（含两个帧头字节），与 tools::get_crc16 / check_crc16 一致。
// mode 的取值：上行 0 空闲 / 1 自瞄 / 2 小符 / 3 大符；
//             下行 0 不控制 / 1 控制云台但不开火 / 2 控制云台且开火。
// ---------------------------------------------------------------------------
struct StandardProtocol
{
  static constexpr const char * name() { return "standard"; }

  // 只有视觉帧，没有导航通道
  static constexpr bool has_nav = false;

  struct __attribute__((packed)) RxFrame
  {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode;
    float q[4];  // wxyz 顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;  // 子弹累计发送次数
    int8_t yaw_offset;
    int8_t pitch_offset;
    uint16_t crc16;
  };

  struct __attribute__((packed)) TxFrame
  {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
  };

  // 字节数钉死：改了就必须同步改电控，否则编不过
  static_assert(sizeof(RxFrame) == 45, "标准协议上行帧必须是 45 字节");
  static_assert(sizeof(TxFrame) == 29, "标准协议下行帧必须是 29 字节");

  static constexpr size_t max_frame_size = 64;

  // ---- 上行帧表：Gimbal 按帧头首字节在这里查 ----
  static constexpr RxFrameSpec rx_frames[] = {
    {'S', 'P', sizeof(RxFrame), RxKind::Vision},
  };
  static constexpr size_t rx_frame_count = 1;

  static bool rx_check(const RxFrameSpec & spec, const uint8_t * frame);
  static void rx_decode(const RxFrameSpec & spec, const uint8_t * frame, RxOut & out);

  // ---- 下行 ----
  static constexpr size_t tx_frame_size = sizeof(TxFrame);
  static void tx_encode(const GimbalCommand & cmd, uint8_t * frame);
};

}  // namespace io

#endif  // IO__GIMBAL_PROTOCOLS_STANDARD_HPP
