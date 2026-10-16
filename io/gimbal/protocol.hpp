#ifndef IO__GIMBAL_PROTOCOL_HPP
#define IO__GIMBAL_PROTOCOL_HPP

#include "io/gimbal/types.hpp"

// ============================================================================
// 云台串口协议的编译期选择
//
// 由 CMake 的 GIMBAL_PROTOCOL 开关定义下面这些宏之一：
//   GIMBAL_PROTOCOL_STANDARD   26 赛季标准协议（重装 / 轮腿步兵 / 狗洞步兵）
//
// 为什么放在编译期：协议是电控那边的东西，一个兵种一旦定下来就不会中途换，
// 编译期选定可以零开销（静态函数直接内联），也不用为用不到的协议付出体积。
//
// ----------------------------------------------------------------------------
// 新增一种协议需要做的：
//   1. 在 gimbal/protocols/ 下加 xxx.hpp + xxx.cpp
//   2. 实现下面这组成员（Gimbal 只依赖这些，缺哪个都会在调用点编译报错）：
//
//        static constexpr const char * name();
//        static constexpr size_t  max_frame_size;   // 一帧最多多少字节
//        static constexpr size_t  rx_head_size;     // 上行帧头字节数
//        static constexpr uint8_t rx_head0, rx_head1;
//        static constexpr size_t  rx_frame_size;    // 上行整帧字节数
//        static bool rx_check(const uint8_t * frame);
//        static void rx_decode(const uint8_t * frame, GimbalFeedback & out);
//        static constexpr size_t tx_frame_size;     // 下行整帧字节数
//        static void tx_encode(const GimbalCommand & cmd, uint8_t * frame);
//
//   3. 在下面加一个分支，并在顶层 CMakeLists 的 GIMBAL_PROTOCOL 取值列表里加一项
//
// 注意：哨兵协议除了视觉帧之外还要传 nav 帧（同一个串口上两种帧头），
// 到时候要把这里的「单个帧头 + 单个帧长」换成一张帧表，
// gimbal.cpp 的 read_thread 里已经用注释标出了分派点。
// ============================================================================

#if defined(GIMBAL_PROTOCOL_STANDARD)
  #include "io/gimbal/protocols/standard.hpp"
namespace io
{
using GimbalProtocol = StandardProtocol;
}
#else
  #error "未指定云台串口协议：请通过 -DGIMBAL_PROTOCOL=<名字> 指定（取值见顶层 CMakeLists）"
#endif

#endif  // IO__GIMBAL_PROTOCOL_HPP
