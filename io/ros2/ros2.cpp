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
constexpr char kRobotStatusTopic[] = "robot_status";
constexpr char kGameStatusTopic[] = "game_status";
constexpr char kRobotBuffTopic[] = "robot_buff";
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
  robot_status_publisher_ =
    node_->create_publisher<sp_msgs::msg::RMUCRobotStatus>(kRobotStatusTopic, 10);
  game_status_publisher_ = node_->create_publisher<sp_msgs::msg::RMUCGameStatus>(kGameStatusTopic, 10);
  robot_buff_publisher_ = node_->create_publisher<sp_msgs::msg::RMUCRobotBuff>(kRobotBuffTopic, 10);

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

void ROS2::publish_referee(const NavFeedback & nav, bool is_detect_enemy)
{
  const auto stamp = node_->now();

  sp_msgs::msg::RMUCGameStatus game;
  game.header.stamp = stamp;
  game.game_progress = nav.game_progress;
  game.stage_remain_time = nav.stage_remain_time;
  game_status_publisher_->publish(game);

  sp_msgs::msg::RMUCRobotStatus robot;
  robot.header.stamp = stamp;
  robot.current_hp = nav.current_hp;
  robot.shooter_heat = nav.shooter_17mm_barrel_heat;
  robot.ammo_allow = nav.projectile_allowance_17mm;
  robot.outpost_hp = nav.outpost_Hp;
  robot.base_hp = nav.base_Hp;
  robot.is_detect_enemy = is_detect_enemy;
  robot_status_publisher_->publish(robot);

  sp_msgs::msg::RMUCRobotBuff buff;
  buff.header.stamp = stamp;
  buff.vulnerability_pct = nav.vulnerability_buff;
  robot_buff_publisher_->publish(buff);
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
