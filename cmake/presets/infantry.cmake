# 步兵（42mm 小弹丸，轮腿 / 狗洞共用）
# 两个步兵的差异只在 configs 里的 yaml，构建配置完全一致
# 需要：自瞄 + 打符
#
#   cmake -B build -DROBOT=infantry
set(BUILD_AUTO_AIM        ON  CACHE BOOL "自瞄核心")
set(BUILD_AUTO_BUFF       ON  CACHE BOOL "打符")
set(BUILD_OMNIPERCEPTION  OFF CACHE BOOL "全向感知（哨兵）")
set(BUILD_NAV             OFF CACHE BOOL "ROS2 通信（哨兵导航 / ROS2 仿真桥）")
set(BUILD_SIM             OFF CACHE BOOL "仿真接口（共享内存 / 仿真 IMU）")
set(BUILD_DEBUG           OFF CACHE BOOL "调试可视化（rerun，需预先安装 SDK）")
set(BUILD_TESTS           OFF CACHE BOOL "测试程序与调试工具")
set(BUILD_CALIBRATION     ON  CACHE BOOL "标定程序")
