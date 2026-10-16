# 仿真
# 编译：自瞄 + 共享内存仿真接口 + rerun 调试可视化
# 不编：打符 / 全向感知 / ROS2 / 标定 / 测试
#
#   cmake -B build -DROBOT=sim
#
# 需要预先安装 rerun_sdk（因为 BUILD_DEBUG=ON）
#
# 注意：ROS2 那条仿真路线（auto_aim_sim）需要 BUILD_NAV，本预设是关的，
# 所以只产出走共享内存的 auto_aim_sim_shm。
set(BUILD_AUTO_AIM        ON  CACHE BOOL "自瞄核心")
set(BUILD_AUTO_BUFF       OFF CACHE BOOL "打符")
set(BUILD_OMNIPERCEPTION  OFF CACHE BOOL "全向感知（哨兵）")
set(BUILD_NAV             OFF CACHE BOOL "ROS2 通信（哨兵导航 / ROS2 仿真桥）")
set(BUILD_SIM             ON  CACHE BOOL "仿真接口（共享内存 / 仿真 IMU）")
set(BUILD_DEBUG           ON  CACHE BOOL "调试可视化（rerun，需预先安装 SDK）")
set(BUILD_TESTS           OFF CACHE BOOL "测试程序与调试工具")
set(BUILD_CALIBRATION     OFF CACHE BOOL "标定程序")
