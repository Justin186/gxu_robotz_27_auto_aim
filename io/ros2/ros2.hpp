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
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
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
//   - 取图接口叫 take_image()，语义明确是"取走后清空"（原来的 get_image()
//     名字是 get、行为是 take，调用方容易重复取到空图）
//   - 图像时间戳记的是本机收到消息的时刻（steady_clock）。原来把 ROS 的
//     system_clock 时间戳直接塞进 steady_clock::time_point，两者时钟起点不同，
//     转出来的时间戳是错的，而这个时间戳会喂给 Perceptron / Solver
//   - 用 cv_bridge 转换图像，不再手写 memcpy（cv_bridge 本来就已是依赖）
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

  // 取走导航那边发来的图像。取到返回 true，并把缓存清空。
  bool take_image(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

  // 丢弃缓存的图像
  void drop_image();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sp_msgs::msg::AutoAimTarget>::SharedPtr target_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<sp_msgs::msg::RMUCSentryCmd>::SharedPtr sentry_cmd_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  mutable std::mutex cmd_vel_mutex_;
  std::optional<geometry_msgs::msg::Twist> latest_cmd_vel_;

  std::atomic<uint8_t> posture_{0};

  std::mutex image_mutex_;
  cv::Mat latest_image_;
  std::chrono::steady_clock::time_point latest_image_stamp_{};

  std::unique_ptr<std::thread> spin_thread_;

  // 是否是本对象完成了 rclcpp 初始化（决定析构时要不要 shutdown）
  bool owns_context_ = false;
};

}  // namespace io

#endif  // IO__ROS2_HPP
