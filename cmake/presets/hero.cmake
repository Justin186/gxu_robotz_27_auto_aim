# 重装（22mm 大弹丸）
# 编译：自瞄 + rerun 调试可视化 + 测试程序 + 标定程序
# 不编：打符 / 全向感知 / ROS2 / 仿真
#
#   cmake -B build -DROBOT=hero
#
# 需要预先安装 rerun_sdk（因为 BUILD_DEBUG=ON）
set(BUILD_AUTO_AIM        ON  CACHE BOOL "自瞄核心")
set(BUILD_AUTO_BUFF       OFF CACHE BOOL "打符")
set(BUILD_OMNIPERCEPTION  OFF CACHE BOOL "全向感知（哨兵）")
set(BUILD_NAV             OFF CACHE BOOL "ROS2 通信（哨兵导航 / ROS2 仿真桥）")
set(BUILD_SIM             OFF CACHE BOOL "仿真接口（共享内存 / 仿真 IMU）")
set(BUILD_DEBUG           ON  CACHE BOOL "调试可视化（rerun，需预先安装 SDK）")
set(BUILD_TESTS           ON  CACHE BOOL "测试程序与调试工具")
set(BUILD_CALIBRATION     ON  CACHE BOOL "标定程序")

# 云台串口（与电控的约定，见 io/gimbal/protocol.hpp / types.hpp）
set(GIMBAL_PROTOCOL       "standard" CACHE STRING "云台串口协议")
set(GIMBAL_HAS_OFFSET     ON  CACHE BOOL "反馈帧带 yaw/pitch 偏置（45 字节；OFF 为 43 字节）")
