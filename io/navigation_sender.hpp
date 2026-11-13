#ifndef IO__NAVIGATION_SENDER_HPP
#define IO__NAVIGATION_SENDER_HPP

#include <cstdint>
#include <string>

#include "io/gimbal/types.hpp"
#include "serial/serial.h"
#include "tools/crc.hpp"

namespace io
{
class NavigationSender
{
public:
  NavigationSender(
    const std::string & port, uint32_t baudrate = 115200,
    tools::CrcByteOrder crc_order = tools::CrcByteOrder::Little);

  void send(const NavCommand & command);
  bool is_open() const;

private:
  tools::CrcByteOrder crc_order_;
  serial::Serial serial_;
};
}  // namespace io

#endif  // IO__NAVIGATION_SENDER_HPP
