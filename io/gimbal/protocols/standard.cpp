#include "io/gimbal/protocols/standard.hpp"

#include <cstring>

#include "tools/crc.hpp"

namespace io
{
bool StandardProtocol::rx_check(const uint8_t * frame)
{
  return tools::check_crc16(frame, rx_frame_size);
}

void StandardProtocol::rx_decode(const uint8_t * frame, GimbalFeedback & out)
{
  RxFrame f;
  std::memcpy(&f, frame, sizeof(f));

  out.q = Eigen::Quaterniond(f.q[0], f.q[1], f.q[2], f.q[3]);
  out.yaw = f.yaw;
  out.yaw_vel = f.yaw_vel;
  out.pitch = f.pitch;
  out.pitch_vel = f.pitch_vel;
  out.bullet_speed = f.bullet_speed;
  out.bullet_count = f.bullet_count;
  out.yaw_offset = f.yaw_offset;
  out.pitch_offset = f.pitch_offset;
  out.raw_mode = f.mode;

  out.mode_valid = true;
  switch (f.mode) {
    case 0:
      out.mode = GimbalMode::IDLE;
      break;
    case 1:
      out.mode = GimbalMode::AUTO_AIM;
      break;
    case 2:
      out.mode = GimbalMode::SMALL_BUFF;
      break;
    case 3:
      out.mode = GimbalMode::BIG_BUFF;
      break;
    default:
      out.mode = GimbalMode::IDLE;
      out.mode_valid = false;
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
