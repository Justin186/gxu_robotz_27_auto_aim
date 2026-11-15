// 自测 io::ROS2 的收发闭环：自己起一个"假导航"节点往 sentry_cmd 发指令，
// 再通过 io::ROS2 把姿态读回来，确认话题名与消息类型两边对得上。
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sp_msgs/msg/rmuc_sentry_cmd.hpp>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  // 假导航端：只负责往 sentry_cmd 发指令
  auto nav_node = std::make_shared<rclcpp::Node>("fake_nav");
  auto publisher = nav_node->create_publisher<sp_msgs::msg::RMUCSentryCmd>("sentry_cmd", 10);
  std::thread([nav_node]() { rclcpp::spin(nav_node); }).detach();

  uint8_t posture = 1;
  for (int i = 0; i < 10 && !exiter.exit(); ++i) {
    sp_msgs::msg::RMUCSentryCmd msg;
    msg.cmd_posture = posture;
    publisher->publish(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tools::logger()->info("published posture={}  read back={}", posture, ros2.sentry_posture());

    posture = (posture == 1) ? 2 : 1;
  }

  return 0;
}
