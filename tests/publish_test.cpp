// 自测：以固定频率向导航发布瞄准目标，用来确认导航那边能收到。
#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  double i = 0;
  while (!exiter.exit()) {
    Eigen::Vector4d target{i, i + 1, 1, auto_aim::ArmorName::sentry + 1};
    ros2.publish_target(target);
    i++;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }
  return 0;
}
