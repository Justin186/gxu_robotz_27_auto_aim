# 哨兵（42mm）
# 编译：自瞄 + 打符 + 全向感知 + ROS2 导航通信 + rerun 调试 + 测试 + 标定
#
#   cmake -B build -DROBOT=sentry
#
# 需要预先安装 rerun_sdk（因为 BUILD_DEBUG=ON）
#
# 注意：src/sentry*.cpp 与 io/ros2/ 目前仍是同济遗留代码，其中引用了并不存在的
# io/ros2/ros2.hpp，所以本预设现在编不过。这部分你们计划重写，重写完再启用。
set(BUILD_AUTO_AIM        ON  CACHE BOOL "自瞄核心")
set(BUILD_AUTO_BUFF       ON  CACHE BOOL "打符")
set(BUILD_OMNIPERCEPTION  ON  CACHE BOOL "全向感知（哨兵）")
set(BUILD_NAV             ON  CACHE BOOL "ROS2 通信（哨兵导航 / ROS2 仿真桥）")
set(BUILD_SIM             OFF CACHE BOOL "仿真接口（共享内存 / 仿真 IMU）")
set(BUILD_DEBUG           ON  CACHE BOOL "调试可视化（rerun，需预先安装 SDK）")
set(BUILD_TESTS           ON  CACHE BOOL "测试程序与调试工具")
set(BUILD_CALIBRATION     ON  CACHE BOOL "标定程序")
