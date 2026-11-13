#include "navigation_sender.hpp"

#include <stdexcept>

#include "io/gimbal/protocols/sentry.hpp"

namespace io
{
NavigationSender::NavigationSender(
  const std::string & port, uint32_t baudrate, tools::CrcByteOrder crc_order)
: crc_order_(crc_order), serial_(port, baudrate)
{
  if (!serial_.isOpen()) {
    throw std::runtime_error("无法打开导航串口: " + port);
  }
}

void NavigationSender::send(const NavCommand & command)
{
  if (!serial_.isOpen()) {
    throw std::runtime_error("导航串口未打开");
  }

  uint8_t frame[SentryProtocol<false>::tx_nav_frame_size]{};
  SentryProtocol<false>::tx_encode_nav(command, frame, crc_order_);
  serial_.write(frame, sizeof(frame));
}

bool NavigationSender::is_open() const
{
  return serial_.isOpen();
}
}  // namespace io
