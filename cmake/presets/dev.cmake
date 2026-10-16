# 开发机：能开的全开（方便一台机器上验证所有兵种的编译）
#
#   cmake -B build -DROBOT=dev
#
# BUILD_DEBUG 默认是关的，因为 rerun_sdk 需要预先安装到系统里。
# 装好之后再追加：
#   cmake -B build -DROBOT=dev -DBUILD_DEBUG=ON
set(BUILD_AUTO_AIM        ON  CACHE BOOL "自瞄核心")
set(BUILD_AUTO_BUFF       ON  CACHE BOOL "打符")
set(BUILD_OMNIPERCEPTION  ON  CACHE BOOL "全向感知（哨兵）")
set(BUILD_NAV             ON  CACHE BOOL "ROS2 通信（哨兵导航 / ROS2 仿真桥）")
set(BUILD_SIM             ON  CACHE BOOL "仿真接口（共享内存 / 仿真 IMU）")
set(BUILD_DEBUG           OFF CACHE BOOL "调试可视化（rerun，需预先安装 SDK）")
set(BUILD_TESTS           ON  CACHE BOOL "测试程序与调试工具")
set(BUILD_CALIBRATION     ON  CACHE BOOL "标定程序")
