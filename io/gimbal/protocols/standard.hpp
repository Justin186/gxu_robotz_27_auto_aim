#ifndef IO__GIMBAL_PROTOCOLS_STANDARD_HPP
#define IO__GIMBAL_PROTOCOLS_STANDARD_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "io/gimbal/frame.hpp"
#include "io/gimbal/types.hpp"
#include "tools/crc.hpp"

namespace io
{
// ---------------------------------------------------------------------------
// 26 赛季标准云台协议（重装 / 轮腿步兵 / 狗洞步兵共用）
//
// 上行帧有**两种布局**，取决于电控有没有实现"稳态补偿偏置"这两个字节：
//
//   带偏置 45 字节
//     'S' 'P' | mode u8 | q[4] f32 | yaw f32 | yaw_vel f32 | pitch f32 |
//     pitch_vel f32 | bullet_speed f32 | bullet_count u16 |
//     yaw_offset i8 | pitch_offset i8 | crc16 u16
//
//   不带偏置 43 字节
//     同上，但少了最后那两个 i8
//
// 由模板参数 HasOffset 在编译期选定 —— 帧长因此不同，**必须和电控一一对应**。
// 选错的症状是整帧错位：收不到帧、或 CRC 一直校验失败。
//
// 下行帧两种布局完全一样（29 字节），不含偏置。
//
// CRC16 覆盖整帧（含两个帧头字节），字节序由 io::Gimbal 传入。
// mode 取值：上行 0 空闲 / 1 自瞄 / 2 小符 / 3 大符；
//           下行 0 不控制 / 1 控制云台但不开火 / 2 控制云台且开火。
// ---------------------------------------------------------------------------
template <bool HasOffset>
struct StandardProtocol
{
  static constexpr bool has_offset = HasOffset;
  static constexpr bool has_nav = false;

  static constexpr const char * name() { return HasOffset ? "standard" : "standard-no-offset"; }

  struct __attribute__((packed)) RxFrameWithOffset
  {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode;   // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];     // wxyz 顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;  // 子弹累计发送次数
    int8_t yaw_offset;      // 单位 0.1 度
    int8_t pitch_offset;    // 单位 0.1 度
    uint16_t crc16;
  };

  struct __attribute__((packed)) RxFrameNoOffset
  {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
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

  // 帧长钉死：改了就必须同步改电控
  static_assert(sizeof(RxFrameWithOffset) == 45, "带偏置的上行帧必须是 45 字节");
  static_assert(sizeof(RxFrameNoOffset) == 43, "不带偏置的上行帧必须是 43 字节");
  static_assert(sizeof(TxFrame) == 29, "标准协议下行帧必须是 29 字节");

  using RxFrame = std::conditional_t<HasOffset, RxFrameWithOffset, RxFrameNoOffset>;

  static constexpr size_t max_frame_size = 64;

  // ---- 上行帧表：Gimbal 按帧头首字节在这里查 ----
  static constexpr RxFrameSpec rx_frames[] = {
    {'S', 'P', sizeof(RxFrame), RxKind::Vision},
  };
  static constexpr size_t rx_frame_count = 1;

  static bool rx_check(const RxFrameSpec & spec, const uint8_t * frame, tools::CrcByteOrder order)
  {
    return tools::check_crc16(frame, spec.size, order);
  }

  static void rx_decode(const RxFrameSpec & spec, const uint8_t * frame, RxOut & out)
  {
    if (spec.kind != RxKind::Vision) return;

    RxFrame f;
    std::memcpy(&f, frame, sizeof(f));

    auto & fb = out.vision;
    fb.q = Eigen::Quaterniond(f.q[0], f.q[1], f.q[2], f.q[3]);
    fb.yaw = f.yaw;
    fb.yaw_vel = f.yaw_vel;
    fb.pitch = f.pitch;
    fb.pitch_vel = f.pitch_vel;
    fb.bullet_speed = f.bullet_speed;
    fb.bullet_count = f.bullet_count;
    fb.raw_mode = f.mode;

    if constexpr (HasOffset) {
      fb.yaw_offset = f.yaw_offset;
      fb.pitch_offset = f.pitch_offset;
    } else {
      // 这块电控的帧里没有偏置，上层自然不做稳态补偿
      fb.yaw_offset = 0;
      fb.pitch_offset = 0;
    }

    fb.mode_valid = true;
    switch (f.mode) {
      case 0:
        fb.mode = GimbalMode::IDLE;
        break;
      case 1:
        fb.mode = GimbalMode::AUTO_AIM;
        break;
      case 2:
        fb.mode = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        fb.mode = GimbalMode::BIG_BUFF;
        break;
      default:
        fb.mode = GimbalMode::IDLE;
        fb.mode_valid = false;
        break;
    }
  }

  // ---- 下行 ----
  static constexpr size_t tx_frame_size = sizeof(TxFrame);

  static void tx_encode(const GimbalCommand & cmd, uint8_t * frame, tools::CrcByteOrder order)
  {
    TxFrame f{};
    f.mode = cmd.control ? (cmd.fire ? 2 : 1) : 0;
    f.yaw = cmd.yaw;
    f.yaw_vel = cmd.yaw_vel;
    f.yaw_acc = cmd.yaw_acc;
    f.pitch = cmd.pitch;
    f.pitch_vel = cmd.pitch_vel;
    f.pitch_acc = cmd.pitch_acc;

    tools::put_crc16(reinterpret_cast<uint8_t *>(&f), sizeof(f) - sizeof(f.crc16), order);

    std::memcpy(frame, &f, sizeof(f));
  }
};

}  // namespace io

#endif  // IO__GIMBAL_PROTOCOLS_STANDARD_HPP
