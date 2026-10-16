#include "ros2.hpp"

namespace io
{
// ============================================================================
// 与导航节点约定的话题名
//
// ★ 集中在这里，等导航那边把接口定下来后改这几个常量即可。
// ============================================================================
namespace
{
constexpr char kTargetTopic[] = "auto_aim_target";
constexpr char kCmdVelTopic[] = "cmd_vel";
constexpr char kSentryCmdTopic[] = "sentry_cmd";
}  // namespace

ROS2::ROS2()
{
  // 调用方的 main 没有调 rclcpp::init，这里按需补上
  if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    owns_context_ = true;
  }

  node_ = std::make_shared<rclcpp::Node>("sentry_ros2");

  // ---------------- 发布 ----------------
  target_publisher_ = node_->create_publisher<sp_msgs::msg::AutoAimTarget>(kTargetTopic, 10);

  // ---------------- 订阅 ----------------
  cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    kCmdVelTopic, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      latest_cmd_vel_ = *msg;
    });

  sentry_cmd_subscription_ = node_->create_subscription<sp_msgs::msg::RMUCSentryCmd>(
    kSentryCmdTopic, 10,
    [this](sp_msgs::msg::RMUCSentryCmd::SharedPtr msg) { posture_.store(msg->cmd_posture); });

  // 订阅回调要靠 spin 驱动
  spin_thread_ = std::make_unique<std::thread>([this]() { rclcpp::spin(node_); });
}

ROS2::~ROS2()
{
  if (owns_context_) {
    // shutdown 之后 rclcpp::spin 会返回，线程随之退出
    rclcpp::shutdown();
  }
  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }
}

void ROS2::publish_target(const Eigen::Vector4d & target, bool tracking)
{
  sp_msgs::msg::AutoAimTarget msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "world";
  msg.x = target[0];
  msg.y = target[1];
  msg.z = target[2];
  msg.yaw = target[3];
  msg.tracking = tracking;

  target_publisher_->publish(msg);
}

std::optional<geometry_msgs::msg::Twist> ROS2::cmd_vel()
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  auto out = latest_cmd_vel_;
  latest_cmd_vel_.reset();
  return out;
}

uint8_t ROS2::sentry_posture() const { return posture_.load(); }

}  // namespace io
