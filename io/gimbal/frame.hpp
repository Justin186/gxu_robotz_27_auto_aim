#ifndef IO__GIMBAL_FRAME_HPP
#define IO__GIMBAL_FRAME_HPP

#include <cstddef>
#include <cstdint>

namespace io
{
// 上行帧的类别。协议自己定义语义，Gimbal 只负责按帧头找到对应的帧，
// 解析完把它交给上面对应的那一半。
//
//   Vision —— 云台反馈（姿态、弹速、模式…），所有兵种都有
//   Nav    —— 导航/比赛状态（血量、姿态指令、位置…），只有哨兵那条通道
enum class RxKind : uint8_t
{
  Vision,
  Nav
};

// 一帧的线上描述：两个帧头字节 + 整帧字节数 + 类别
struct RxFrameSpec
{
  uint8_t head0;
  uint8_t head1;
  size_t size;
  RxKind kind;
};

}  // namespace io

#endif  // IO__GIMBAL_FRAME_HPP
