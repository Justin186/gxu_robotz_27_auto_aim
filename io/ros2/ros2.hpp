#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

#include "rm_interfaces/msg/target.hpp"

namespace io
{
// 哨兵与导航节点之间的 ROS2 通信层。
//
// 话题名与消息类型集中在 ros2.cpp 顶部的常量里，等导航那边把接口定下来之后，
// 只需要改那几个常量和对 publish() 的字段映射，不用动这里的调用方。
//
// 说明：ROS2 需要 rclcpp 初始化，但调用方（src/sentry*.cpp、tests/*_test.cpp）
// 的 main 都没有调 rclcpp::init，所以本类在构造时按需初始化、析构时按需关闭。
// 一个进程里请只创建一个 ROS2 对象。
class ROS2
{
public:
  ROS2();
  ~ROS2();

  // ---- 订阅 ----

  // 裁判系统给出的"无敌"机器人 ID（哨兵据此跳过不该打的装甲）
  std::vector<int8_t> subscribe_enemy_status();

  // 导航指定的优先目标 ID
  std::vector<int8_t> subscribe_autoaim_target();

  // ---- 发布 ----

  // 当前瞄准目标 (x, y, z, yaw)
  void publish(const Eigen::Vector4d & target_info);

  // 通用发布器创建，供测试程序自建话题用
  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr create_publisher(
    const std::string & node_name, const std::string & topic, int queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);
    extra_nodes_.push_back(node);
    return node->create_publisher<MsgT>(topic, queue_size);
  }

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr enemy_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr autoaim_target_subscription_;

  std::mutex mutex_;
  std::vector<int8_t> enemy_status_;
  std::vector<int8_t> autoaim_target_;

  std::vector<rclcpp::Node::SharedPtr> extra_nodes_;
  std::unique_ptr<std::thread> spin_thread_;

  // 是否是本对象完成了 rclcpp 初始化（决定析构时要不要 shutdown）
  bool owns_context_ = false;
};

}  // namespace io

#endif  // IO__ROS2_HPP
