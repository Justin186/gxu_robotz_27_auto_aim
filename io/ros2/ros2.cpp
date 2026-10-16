#include "ros2.hpp"

#include <cv_bridge/cv_bridge.h>

#include "tools/logger.hpp"

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
constexpr char kImageTopic[] = "image";
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
  target_publisher_ = node_->create_publisher<std_msgs::msg::String>(kTargetTopic, 10);

  // ---------------- 订阅 ----------------
  cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    kCmdVelTopic, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      latest_cmd_vel_ = *msg;
    });

  sentry_cmd_subscription_ = node_->create_subscription<sp_msgs::msg::RMUCSentryCmd>(
    kSentryCmdTopic, 10,
    [this](sp_msgs::msg::RMUCSentryCmd::SharedPtr msg) { posture_.store(msg->cmd_posture); });

  image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
    kImageTopic, 10, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      cv::Mat img;
      try {
        img = cv_bridge::toCvCopy(msg, "bgr8")->image;
      } catch (const cv_bridge::Exception & e) {
        tools::logger()->warn("ROS2 image convert failed: {}", e.what());
        return;
      }

      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_ = std::move(img);
      // ROS 的 header.stamp 是 system_clock（墙上时钟），本项目内部统一用
      // steady_clock（单调时钟），两者起点不同、不能直接互转。所以这里记录
      // 本机收到消息的时刻，语义与 io::Camera 给出的时间戳一致。
      latest_image_stamp_ = std::chrono::steady_clock::now();
    });

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

void ROS2::publish_target(const Eigen::Vector4d & target)
{
  // TODO(Q1)：目前仍沿用 26 赛季的"逗号字符串"约定，以免导航那边收不到。
  //   等确定改用 sp_msgs/AutoAimTarget 结构体后，把这里换掉。
  //   已知问题：std::to_string(double) 只有 6 位小数，位置精度有损失。
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = std::to_string(target[0]) + "," + std::to_string(target[1]) + "," +
              std::to_string(target[2]) + "," + std::to_string(target[3]);
  target_publisher_->publish(*msg);
}

std::optional<geometry_msgs::msg::Twist> ROS2::cmd_vel()
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  auto out = latest_cmd_vel_;
  latest_cmd_vel_.reset();
  return out;
}

uint8_t ROS2::sentry_posture() const { return posture_.load(); }

bool ROS2::take_image(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (latest_image_.empty()) return false;

  img = std::move(latest_image_);
  latest_image_.release();
  timestamp = latest_image_stamp_;
  return true;
}

void ROS2::drop_image()
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  latest_image_.release();
}

}  // namespace io
