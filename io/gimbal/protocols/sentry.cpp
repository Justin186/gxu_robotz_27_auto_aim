#include "io/gimbal/protocols/sentry.hpp"

#include <cstring>

#include "tools/crc.hpp"

namespace io
{
bool SentryProtocol::rx_check(
  const RxFrameSpec & spec, const uint8_t * frame, tools::CrcByteOrder order)
{
  return tools::check_crc16(frame, spec.size, order);
}

void SentryProtocol::rx_decode(const RxFrameSpec & spec, const uint8_t * frame, RxOut & out)
{
  if (spec.kind == RxKind::Vision) {
    // 视觉帧与标准协议完全一致，直接复用
    StandardProtocol::rx_decode(spec, frame, out);
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

void SentryProtocol::tx_encode(
  const GimbalCommand & cmd, uint8_t * frame, tools::CrcByteOrder order)
{
  StandardProtocol::tx_encode(cmd, frame, order);
}

void SentryProtocol::tx_encode_nav(
  const NavCommand & cmd, uint8_t * frame, tools::CrcByteOrder order)
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

}  // namespace io
