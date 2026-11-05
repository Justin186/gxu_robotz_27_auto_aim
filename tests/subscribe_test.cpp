// 自测：打印导航下发的内容（底盘速度指令 + 哨兵姿态指令），用来确认视觉这边收得到。
#include <chrono>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  while (!exiter.exit()) {
    // cmd_vel 取走后会被清空，没有新指令时返回 nullopt
    if (auto v = ros2.cmd_vel(); v.has_value()) {
      tools::logger()->info(
        "cmd_vel: linear=({:.3f}, {:.3f}, {:.3f}) angular=({:.3f}, {:.3f}, {:.3f})", v->linear.x,
        v->linear.y, v->linear.z, v->angular.x, v->angular.y, v->angular.z);
    }

    if (auto posture = ros2.sentry_posture(); posture != 0) {
      tools::logger()->info("sentry posture: {}", posture);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}
