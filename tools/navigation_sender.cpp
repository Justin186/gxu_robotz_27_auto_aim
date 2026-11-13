#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "io/navigation_sender.hpp"
#include "tools/exiter.hpp"

namespace
{
void usage(const char * program)
{
  std::cerr
    << "用法: " << program
    << " <串口> <linear_x> <linear_y> <linear_z> <angular_x> <angular_y> <angular_z>"
       " <move_mode> [baudrate] [rate_hz] [little|big]\n"
    << "示例: " << program << " /dev/ttyACM0 0 0 0 0 0 0 3 115200 20 little\n";
}

template <typename T>
T parse(const char * text, const char * name)
{
  try {
    size_t pos = 0;
    const auto value = std::stod(text, &pos);
    if (text[pos] != '\0') {
      throw std::invalid_argument("包含无效字符");
    }
    return static_cast<T>(value);
  } catch (const std::exception & error) {
    throw std::invalid_argument(std::string(name) + " 无效: " + error.what());
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 9 || argc > 12) {
    usage(argv[0]);
    return 2;
  }

  try {
    io::NavCommand command;
    command.linear_x = parse<float>(argv[2], "linear_x");
    command.linear_y = parse<float>(argv[3], "linear_y");
    command.linear_z = parse<float>(argv[4], "linear_z");
    command.angular_x = parse<float>(argv[5], "angular_x");
    command.angular_y = parse<float>(argv[6], "angular_y");
    command.angular_z = parse<float>(argv[7], "angular_z");
    command.move_mode = parse<uint8_t>(argv[8], "move_mode");
    if (command.move_mode < 1 || command.move_mode > 3) {
      throw std::invalid_argument("move_mode 必须是 1、2 或 3");
    }

    uint32_t baudrate = argc >= 10 ? parse<uint32_t>(argv[9], "baudrate") : 115200;
    const double rate_hz = argc >= 11 ? parse<double>(argv[10], "rate_hz") : 20.0;
    if (baudrate == 0 || rate_hz <= 0) {
      throw std::invalid_argument("baudrate 和 rate_hz 必须大于 0");
    }

    auto crc_order = tools::CrcByteOrder::Little;
    if (argc == 12) {
      const std::string order = argv[11];
      if (order == "big") {
        crc_order = tools::CrcByteOrder::Big;
      } else if (order != "little") {
        throw std::invalid_argument("CRC 字节序只能是 little 或 big");
      }
    }

    io::NavigationSender sender(argv[1], baudrate, crc_order);
    tools::Exiter exiter;
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    std::cout << "导航帧发送中，按 Ctrl+C 停止；帧率 " << rate_hz << " Hz\n";
    while (!exiter.exit()) {
      sender.send(command);
      std::this_thread::sleep_for(period);
    }
  } catch (const std::exception & error) {
    std::cerr << "navigation_sender: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
