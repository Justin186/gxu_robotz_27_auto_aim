#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sp_msgs/msg/auto_aim_target.hpp>
#include <sp_msgs/msg/rmuc_sentry_cmd.hpp>

namespace io
{
// 哨兵与导航之间的 ROS2 通信层。
//
// 与 26 赛季那版的区别：
//   - 只起一个 rclcpp::Node（原来 publish2nav / subscribe2nav / 自身一共三个，
//     而且 publish2nav.cpp 里还把 shared_ptr 赋成 0）
//   - 瞄准目标改用 sp_msgs/AutoAimTarget 结构体，不再把 4 个 double 拼成
//     逗号字符串（那样会丢精度，且字段顺序无类型约束）
//   - 不再从 ROS 取图（26 赛季订阅 /odin1/image 那套已不需要），
//     因此原来那个把 system_clock 时间戳硬塞进 steady_clock 的错误也一并消失；
//     主相机仍走 io::Camera，侧向相机走 io::USBCamera
//   - cmd_vel() 取走后清空、无新指令返回 nullopt，超时交给调用方判断；
//     原来是在回调里挂一个 1500ms 的 timer 去清队列，既绕又脆弱
//
// 话题名集中在 ros2.cpp 顶部，便于和导航那边对齐。
class ROS2
{
public:
  ROS2();
  ~ROS2();

  // ---------------- 发往导航 ----------------

  // 当前瞄准目标 (x, y, z, yaw)，世界坐标系
  // tracking=false 表示本帧无有效目标
  void publish_target(const Eigen::Vector4d & target, bool tracking = true);

  // ---------------- 来自导航 ----------------

  // 底盘速度指令。取走之后会被清空；没有新指令时返回 nullopt。
  std::optional<geometry_msgs::msg::Twist> cmd_vel();

  // 哨兵姿态指令（裁判系统 0x0120）：1=进攻 2=防御 3=移动
  uint8_t sentry_posture() const;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sp_msgs::msg::AutoAimTarget>::SharedPtr target_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<sp_msgs::msg::RMUCSentryCmd>::SharedPtr sentry_cmd_subscription_;

  mutable std::mutex cmd_vel_mutex_;
  std::optional<geometry_msgs::msg::Twist> latest_cmd_vel_;

  std::atomic<uint8_t> posture_{0};

  std::unique_ptr<std::thread> spin_thread_;

  // 是否是本对象完成了 rclcpp 初始化（决定析构时要不要 shutdown）
  bool owns_context_ = false;
};

}  // namespace io

#endif  // IO__ROS2_HPP
