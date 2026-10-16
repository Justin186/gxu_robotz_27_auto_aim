# 哨兵（42mm）
# 需要：自瞄 + 打符 + 全向感知 + ROS2 导航通信
# 注意：哨兵的后置相机走 configs/camera.yaml（mindvision），所以两个相机后端都要开
#
#   cmake -B build -DROBOT=sentry
set(BUILD_AUTO_AIM        ON  CACHE BOOL "自瞄核心")
set(BUILD_AUTO_BUFF       ON  CACHE BOOL "打符")
set(BUILD_OMNIPERCEPTION  ON  CACHE BOOL "全向感知（哨兵）")
set(BUILD_NAV             ON  CACHE BOOL "ROS2 通信（哨兵导航 / ROS2 仿真桥）")
set(BUILD_SIM             OFF CACHE BOOL "仿真接口（共享内存 / 仿真 IMU）")
set(BUILD_DEBUG           OFF CACHE BOOL "调试可视化（rerun，需预先安装 SDK）")
set(BUILD_TESTS           OFF CACHE BOOL "测试程序与调试工具")
set(BUILD_CALIBRATION     ON  CACHE BOOL "标定程序")
set(IO_HIKROBOT           ON  CACHE BOOL "海康相机（前置）")
set(IO_MINDVISION         ON  CACHE BOOL "迈德威视相机（后置）")
