#include "ros2.hpp"

#include <utility>

#include "tools/logger.hpp"

namespace io
{
// ============================================================================
// 与导航节点约定的话题名
//
// ★ 这几个名字是占位，等导航那边定了接口之后改这里即可。
// ============================================================================
namespace
{
constexpr char kEnemyStatusTopic[] = "enemy_status";
constexpr char kAutoAimTargetTopic[] = "autoaim_target";
constexpr char kTargetInfoTopic[] = "target_info";
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

  // 发布：瞄准目标
  target_publisher_ = node_->create_publisher<rm_interfaces::msg::Target>(kTargetInfoTopic, 10);

  // 订阅：无敌敌人 ID
  enemy_status_subscription_ = node_->create_subscription<std_msgs::msg::Int8MultiArray>(
    kEnemyStatusTopic, 10, [this](std_msgs::msg::Int8MultiArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      enemy_status_ = msg->data;
    });

  // 订阅：导航指定的优先目标
  autoaim_target_subscription_ = node_->create_subscription<std_msgs::msg::Int8MultiArray>(
    kAutoAimTargetTopic, 10, [this](std_msgs::msg::Int8MultiArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      autoaim_target_ = msg->data;
    });

  // 订阅回调需要有人 spin，这里起一条后台线程
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

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enemy_status_;
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return autoaim_target_;
}

void ROS2::publish(const Eigen::Vector4d & target_info)
{
  rm_interfaces::msg::Target msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "gimbal";
  msg.tracking = true;
  msg.position.x = target_info[0];
  msg.position.y = target_info[1];
  msg.position.z = target_info[2];
  msg.yaw = target_info[3];

  target_publisher_->publish(msg);
}

}  // namespace io
