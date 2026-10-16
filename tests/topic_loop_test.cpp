// 自测 io::ROS2 的收发闭环：自己发布"无敌敌人 ID"，再通过 ROS2 订阅读回来。
//
// 说明：原来的版本用的是 rm_interfaces::msg::EnemyStatusMsg，但 rm_interfaces
// 里并没有这个消息（全仓库只有这一处引用），所以改用标准消息
// std_msgs/msg/Int8MultiArray。等导航接口定下来后按需调整。
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  auto publisher = ros2.create_publisher<std_msgs::msg::Int8MultiArray>("temp_node", "enemy_status", 10);

  int i = 0;
  while (!exiter.exit()) {
    std_msgs::msg::Int8MultiArray msg;
    msg.data = {1, 2, 3};
    publisher->publish(msg);

    i++;
    if (i % 3 == 0) {
      auto ids = ros2.subscribe_enemy_status();
      tools::logger()->info("invincible enemy ids size: {}", ids.size());
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }

  return 0;
}
