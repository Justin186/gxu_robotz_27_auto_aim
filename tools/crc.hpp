#ifndef TOOLS__CRC_HPP
#define TOOLS__CRC_HPP

#include <cstdint>

namespace tools
{
// CRC 字段在帧尾两字节里的排列顺序。
// 不同电控的实现不一样：把 CRC 塞进 packed 结构体再整体发送的（x86 / ARM 都是
// 小端）就是 Little；逐字节按高字节优先拼帧的是 Big。
enum class CrcByteOrder
{
  Little,  // 低字节在前
  Big      // 高字节在前
};

// len不包括crc8
uint8_t get_crc8(const uint8_t * data, uint16_t len);

// len包括crc8
bool check_crc8(const uint8_t * data, uint16_t len);

// len不包括crc16
uint16_t get_crc16(const uint8_t * data, uint32_t len);

// len包括crc16，按指定字节序解读帧尾两字节
bool check_crc16(const uint8_t * data, uint32_t len, CrcByteOrder order);

// len包括crc16，等价于 CrcByteOrder::Big。
// 保留是为了不打断 26 赛季那批按"大端"约定写的调用点；
// gimbal 那一层已经改成显式传字节序，不要再新增对它的依赖。
bool check_crc16(const uint8_t * data, uint32_t len);

// 把算好的 CRC 按指定字节序写到 data[len] 和 data[len + 1]（len 不包括 crc16）
void put_crc16(uint8_t * data, uint32_t len, CrcByteOrder order);

// 只读取帧尾的 CRC 字段值，不做校验（用于诊断日志）
uint16_t get_crc16_field(const uint8_t * data, uint32_t len, CrcByteOrder order);

}  // namespace tools

#endif  // TOOLS__CRC_HPP
