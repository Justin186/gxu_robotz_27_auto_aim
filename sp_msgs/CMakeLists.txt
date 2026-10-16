cmake_minimum_required(VERSION 3.8)
project(sp_msgs)

# ============================================================================
# sp_msgs —— 哨兵与导航之间的自定义 ROS 2 消息
#
# 只提交 msg/*.msg，生成物不入库。构建方式见 readme：
#
#   colcon build --packages-select sp_msgs --base-paths sp_msgs \
#                --install-base sp_msgs_install --build-base sp_msgs_build
#
# 新增消息时：把 .msg 放进 msg/，加到下面的列表里，重新跑一次上面的命令。
# ============================================================================

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float32Stamped.msg"
  "msg/RMUCEnemyTracks.msg"
  "msg/RMUCGameStatus.msg"
  "msg/RMUCNavControlCmd.msg"
  "msg/RMUCRobotBuff.msg"
  "msg/RMUCRobotControl.msg"
  "msg/RMUCRobotPosition.msg"
  "msg/RMUCRobotStatus.msg"
  "msg/RMUCSentryCmd.msg"
  "msg/RMUCSentryDecisionStatus.msg"
  "msg/RMUCTeamHP.msg"
  "msg/RMUL.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
