#ifndef IO__GIMBAL_PROTOCOL_HPP
#define IO__GIMBAL_PROTOCOL_HPP

#include "io/gimbal/frame.hpp"
#include "io/gimbal/types.hpp"

// ============================================================================
// 云台串口协议的编译期选择
//
// 两个维度：
//
//   GIMBAL_PROTOCOL     协议本身
//     STANDARD   26 赛季标准协议（重装 / 轮腿步兵 / 狗洞步兵共用）
//     SENTRY     标准协议的视觉帧 + 一条导航通道（哨兵）
//
//   GIMBAL_HAS_OFFSET   反馈帧里带不带 yaw/pitch 稳态补偿偏置（默认 ON）
//     ON         上行帧 45 字节，偏置可用
//     OFF        上行帧 43 字节（少两个 i8），偏置恒为 0
//                ★ 电控没实现这两个字节时必须关掉，否则整帧错位
//
// 为什么放在编译期：协议是电控那边的东西，一个兵种定下来就不会中途换；而且
// 帧长不同意味着内存布局不同，运行期变量表达不了。
//
// ----------------------------------------------------------------------------
// 协议实现需要提供（Gimbal 只依赖这些，缺哪个都会在调用点编译报错）：
//
//   static constexpr const char * name();
//   static constexpr bool   has_offset;        // 上行帧带不带偏置
//   static constexpr bool   has_nav;           // 有没有导航通道
//   static constexpr size_t max_frame_size;
//   static constexpr RxFrameSpec rx_frames[];  // 上行帧表（帧头 + 帧长 + 类别）
//   static constexpr size_t rx_frame_count;
//   static bool rx_check(const RxFrameSpec &, const uint8_t * f, tools::CrcByteOrder);
//   static void rx_decode(const RxFrameSpec &, const uint8_t * f, RxOut & out);
//   static constexpr size_t tx_frame_size;
//   static void tx_encode(const GimbalCommand &, uint8_t * f, tools::CrcByteOrder);
//
//   has_nav 为 true 时还要：
//   static constexpr size_t tx_nav_frame_size;
//   static void tx_encode_nav(const NavCommand &, uint8_t * f, tools::CrcByteOrder);
//
// CRC 字节序由 io::Gimbal 从配置里读出来传进来，协议实现不要自己写死。
// ============================================================================

#if !defined(GIMBAL_HAS_OFFSET)
  #error "未指定 GIMBAL_HAS_OFFSET：请用 CMake 的 -DGIMBAL_HAS_OFFSET=ON/OFF 指定"
#endif

#if defined(GIMBAL_PROTOCOL_STANDARD)
  #include "io/gimbal/protocols/standard.hpp"
namespace io
{
using GimbalProtocol = StandardProtocol<GIMBAL_HAS_OFFSET != 0>;
}
#elif defined(GIMBAL_PROTOCOL_SENTRY)
  #include "io/gimbal/protocols/sentry.hpp"
namespace io
{
using GimbalProtocol = SentryProtocol<GIMBAL_HAS_OFFSET != 0>;
}
#else
  #error "未指定云台串口协议：请通过 -DGIMBAL_PROTOCOL=<名字> 指定（取值见顶层 CMakeLists）"
#endif

#endif  // IO__GIMBAL_PROTOCOL_HPP
