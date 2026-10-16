#include "io/gimbal/protocols/standard.hpp"

#include <cstring>

#include "tools/crc.hpp"

namespace io
{
bool StandardProtocol::rx_check(const RxFrameSpec & spec, const uint8_t * frame)
{
  return tools::check_crc16(frame, spec.size);
}

void StandardProtocol::rx_decode(const RxFrameSpec & spec, const uint8_t * frame, RxOut & out)
{
  // 本协议只有视觉帧
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
  fb.yaw_offset = f.yaw_offset;
  fb.pitch_offset = f.pitch_offset;
  fb.raw_mode = f.mode;

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

void StandardProtocol::tx_encode(const GimbalCommand & cmd, uint8_t * frame)
{
  TxFrame f{};
  f.mode = cmd.control ? (cmd.fire ? 2 : 1) : 0;
  f.yaw = cmd.yaw;
  f.yaw_vel = cmd.yaw_vel;
  f.yaw_acc = cmd.yaw_acc;
  f.pitch = cmd.pitch;
  f.pitch_vel = cmd.pitch_vel;
  f.pitch_acc = cmd.pitch_acc;
  f.crc16 = tools::get_crc16(reinterpret_cast<uint8_t *>(&f), sizeof(f) - sizeof(f.crc16));

  std::memcpy(frame, &f, sizeof(f));
}

}  // namespace io
