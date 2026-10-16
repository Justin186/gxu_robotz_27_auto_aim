cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# ============================================================================
# 兵种预设（可选，只是给下面的开关填默认值）
#
#   cmake -B build -DROBOT=hero          # 重装
#   cmake -B build -DROBOT=infantry      # 步兵（轮腿 / 狗洞共用）
#   cmake -B build -DROBOT=sentry        # 哨兵
#   cmake -B build -DROBOT=sim           # 仿真
#   cmake -B build -DROBOT=dev           # 开发机（全开）
#   cmake -B build                       # 默认组合
#
# 命令行显式指定的 -DBUILD_xxx=ON/OFF 优先于预设。
# 想完全手动控制就用 -DROBOT=custom。
# ============================================================================

# 自动发现 cmake/presets/ 下的所有 *.cmake：
# 新增一个兵种预设只需往那个目录丢一个文件，这里不用改。
file(GLOB _preset_names RELATIVE "${CMAKE_SOURCE_DIR}/cmake/presets"
     "${CMAKE_SOURCE_DIR}/cmake/presets/*.cmake")
list(TRANSFORM _preset_names REPLACE "\\.cmake$" "")

set(ROBOT "custom" CACHE STRING "兵种预设（可选值见 cmake/presets/ 目录）")
set_property(CACHE ROBOT PROPERTY STRINGS custom ${_preset_names})

# 记录上一次应用过的兵种。CMake 的 CACHE 变量一旦存在，set(X ... CACHE ...) 就
# 不会再覆盖它 —— 所以同一个 build 目录里把 -DROBOT 从 A 改成 B 时，预设不会
# 生效，而且不报错，会静默地继续按 A 编。这里显式处理这种切换。
#
# 注意：INTERNAL 类型的缓存项每次 set 都会覆盖，所以必须先判断是否已存在，
# 否则这行会把上一次的记录清空，切换检测永远不触发。
if(NOT DEFINED SP_APPLIED_ROBOT)
  set(SP_APPLIED_ROBOT "" CACHE INTERNAL "上一次应用过的兵种预设")
endif()

if(NOT ROBOT STREQUAL "custom")
  set(_preset "${CMAKE_SOURCE_DIR}/cmake/presets/${ROBOT}.cmake")
  if(NOT EXISTS "${_preset}")
    message(FATAL_ERROR "未知兵种预设 '${ROBOT}'，可选: custom;${_preset_names}")
  endif()

  if(NOT SP_APPLIED_ROBOT STREQUAL "" AND NOT SP_APPLIED_ROBOT STREQUAL ROBOT)
    # 换兵种了：先清掉上一次预设写进 cache 的值，
    # 下面 include 时才会重新按新预设的默认值写入。
    message(STATUS "兵种预设由 '${SP_APPLIED_ROBOT}' 切换为 '${ROBOT}'，重置相关开关")
    message(STATUS "  注意：命令行里附带的 -DBUILD_xxx 覆盖也会被重置；如需要请再跑一次 cmake")
    message(STATUS "  更好的做法是用独立目录：cmake --preset ${ROBOT}")
    foreach(
      _v
      BUILD_AUTO_AIM
      BUILD_AUTO_BUFF
      BUILD_OMNIPERCEPTION
      BUILD_NAV
      BUILD_SIM
      BUILD_DEBUG
      BUILD_TESTS
      BUILD_CALIBRATION
      GIMBAL_PROTOCOL)
      unset(${_v} CACHE)
    endforeach()
  endif()

  include("${_preset}")
  set(SP_APPLIED_ROBOT "${ROBOT}" CACHE INTERNAL "上一次应用过的兵种预设" FORCE)
  message(STATUS "--------------------应用兵种预设: ${ROBOT}--------------------")
endif()

# ============================================================================
# 功能开关
# ============================================================================
option(BUILD_AUTO_AIM       "自瞄核心"                            ON)
option(BUILD_AUTO_BUFF      "打符"                                ON)
option(BUILD_OMNIPERCEPTION "全向感知（哨兵）"                     OFF)
option(BUILD_NAV            "ROS2 通信（哨兵导航 / ROS2 仿真桥）"    OFF)
option(BUILD_SIM            "仿真接口（共享内存 / 仿真 IMU）"        OFF)
option(BUILD_DEBUG          "调试可视化（rerun，需预先安装 SDK）"    OFF)
option(BUILD_TESTS          "测试程序与调试工具"                    OFF)
option(BUILD_CALIBRATION    "标定程序"                             ON)

# ---- 云台串口协议（编译期选择，见 io/gimbal/protocol.hpp）----
# 目前只有 26 赛季标准协议，重装 / 轮腿步兵 / 狗洞步兵共用。
# 哨兵那套（同一个串口上多一条 nav 帧通道）会在 GimbalNode 那一步一起加进来。
set(GIMBAL_PROTOCOL "standard" CACHE STRING "云台串口协议")
set_property(CACHE GIMBAL_PROTOCOL PROPERTY STRINGS standard)

# ---- 开关之间的依赖约束：早报错，别拖到链接阶段 ----
if(BUILD_AUTO_BUFF AND NOT BUILD_AUTO_AIM)
  message(FATAL_ERROR "BUILD_AUTO_BUFF 依赖 BUILD_AUTO_AIM（auto_buff 链接了 auto_aim）")
endif()
if(BUILD_OMNIPERCEPTION AND NOT BUILD_AUTO_AIM)
  message(FATAL_ERROR "BUILD_OMNIPERCEPTION 依赖 BUILD_AUTO_AIM")
endif()
if(BUILD_NAV AND NOT BUILD_AUTO_AIM)
  message(FATAL_ERROR "BUILD_NAV（哨兵）依赖 BUILD_AUTO_AIM")
endif()

# ============================================================================
# 基础依赖（任何配置都需要）
# ============================================================================
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# ============================================================================
# 重依赖：按需查找，不用就不找
# ============================================================================

# ---- OpenVINO：自瞄 / 打符 / 全向感知 的推理都要用 ----
if(BUILD_AUTO_AIM OR BUILD_AUTO_BUFF OR BUILD_OMNIPERCEPTION)
  # 不写死版本和路径，交给 find_package 走标准搜索：
  #   - apt 安装的会落在 /usr/lib/cmake/openvino<版本>/
  #   - Intel 官方压缩包解压后在 /opt/intel/<版本>/runtime/cmake/ 也能被找到
  # 特殊情况用 -DOpenVINO_DIR=<你的路径> 指定
  find_package(OpenVINO REQUIRED)
  message(STATUS "OpenVINO: ${OpenVINO_DIR}")
endif()

# ---- Ceres：只有打符的弹道求解用 ----
if(BUILD_AUTO_BUFF)
  find_package(Ceres REQUIRED)
endif()

# ---- rerun：只有调试可视化用（需要预先安装，见 readme）----
if(BUILD_DEBUG)
  find_package(rerun_sdk REQUIRED)
endif()

# ---- ROS2：只有 BUILD_NAV 时才需要 ----
if(BUILD_NAV)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(rosidl_typesupport_cpp REQUIRED)
  set(rm_interfaces_DIR ${CMAKE_SOURCE_DIR}/rm_interfaces/share/rm_interfaces/cmake)
  find_package(rm_interfaces REQUIRED)

  # ---- sp_msgs：本仓库的源码包，需要先用 colcon 构建（见 readme）----
  # 生成物不入库；构造一次即可，之后改 .msg 再重跑那条 colcon 命令。
  list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/sp_msgs_install/sp_msgs")
  find_package(sp_msgs QUIET)
  if(NOT sp_msgs_FOUND)
    message(FATAL_ERROR
      "找不到 sp_msgs。它是本仓库的源码包，请先构建：\n"
      "  colcon build --packages-select sp_msgs --base-paths sp_msgs \\\n"
      "               --install-base sp_msgs_install --build-base sp_msgs_build")
  endif()

  message(STATUS "ROS2: 已启用导航通信相关代码")
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# ============================================================================
# 子目录
# ============================================================================
add_subdirectory(tools)
add_subdirectory(io)

if(BUILD_AUTO_AIM)
  add_subdirectory(tasks/auto_aim)
endif()
if(BUILD_AUTO_BUFF)
  add_subdirectory(tasks/auto_buff)
endif()
if(BUILD_OMNIPERCEPTION)
  add_subdirectory(tasks/omniperception)
endif()

# ============================================================================
# 可执行文件
# ============================================================================

# ---- 自瞄主程序（重装 / 步兵 / 哨兵）----
if(BUILD_AUTO_AIM)
  add_executable(standard src/standard.cpp)
  target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

  add_executable(auto_aim_mpc src/auto_aim_mpc.cpp)
  target_link_libraries(auto_aim_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

  add_executable(uav_debug src/uav_debug.cpp)
  target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
endif()

# ---- 需要同时用到 自瞄 + 打符 的程序 ----
if(BUILD_AUTO_AIM AND BUILD_AUTO_BUFF)
  add_executable(mt_standard src/mt_standard.cpp)
  target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)

  add_executable(standard_mpc src/standard_mpc.cpp)
  target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)

  add_executable(uav src/uav.cpp)
  target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
endif()

# ---- 打符 ----
if(BUILD_AUTO_BUFF)
  add_executable(auto_buff_debug src/auto_buff_debug.cpp)
  target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})

  add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
  target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
endif()

# ---- 仿真 ----
if(BUILD_SIM)
  add_executable(auto_aim_sim_shm src/auto_aim_sim_shm.cpp)
  target_link_libraries(auto_aim_sim_shm ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

  if(BUILD_NAV)
    add_executable(auto_aim_sim src/auto_aim_sim.cpp)
    target_link_libraries(auto_aim_sim ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
  endif()
endif()

# ---- 哨兵 ----
if(BUILD_NAV AND BUILD_OMNIPERCEPTION)
  add_executable(sentry src/sentry.cpp)
  add_executable(sentry_bp src/sentry_bp.cpp)
  add_executable(sentry_debug src/sentry_debug.cpp)
  add_executable(sentry_multithread src/sentry_multithread.cpp)

  foreach(tgt sentry sentry_bp sentry_debug sentry_multithread)
    target_link_libraries(${tgt} ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
  endforeach()
endif()

# ---- 调试可视化（rerun）----
if(BUILD_DEBUG)
  if(BUILD_AUTO_AIM)
    add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
    target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io rerun_sdk)

    add_executable(auto_aim_debug_offline src/auto_aim_debug_offline.cpp)
    target_link_libraries(auto_aim_debug_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io rerun_sdk)
  endif()
endif()

# ============================================================================
# 标定程序
# ============================================================================
if(BUILD_CALIBRATION)
  add_executable(capture calibration/capture.cpp)
  add_executable(calibrate_camera calibration/calibrate_camera.cpp)
  add_executable(calibrate_camera_chessboard calibration/calibrate_camera_chessboard.cpp)
  add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
  add_executable(calibrate_handeye_chessboard calibration/calibrate_handeye_chessboard.cpp)
  add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
  add_executable(split_video calibration/split_video.cpp)
  add_executable(ypr_to_rotation_matrix calibration/ypr_to_rotation_matrix.cpp)

  target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
  target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
  target_link_libraries(calibrate_camera_chessboard ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
  target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
  target_link_libraries(calibrate_handeye_chessboard ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
  target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
  target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)
  target_link_libraries(ypr_to_rotation_matrix ${OpenCV_LIBS} fmt::fmt tools)

  # capture_gimbal 用真相机 + 真云台（configs/calibration.yaml），与仿真无关。
  # 26 赛季 sim 分支曾把它改成读共享内存，这里恢复成 main 的版本。
  add_executable(capture_gimbal calibration/capture_gimbal.cpp)
  target_link_libraries(capture_gimbal ${OpenCV_LIBS} fmt::fmt tools io)
endif()

# ============================================================================
# 测试与调试工具
# ============================================================================
if(BUILD_TESTS)
  # ---- 只需要 tools + io ----
  foreach(tgt camera_test cboard_test fire_test gimbal_response_test
              multi_usbcamera_test usbcamera_test)
    add_executable(${tgt} tests/${tgt}.cpp)
    target_link_libraries(${tgt} ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
  endforeach()

  # ---- 需要 auto_aim ----
  if(BUILD_AUTO_AIM)
    foreach(tgt auto_aim_test camera_detect_test camera_detect_distance_test
                camera_thread_test detector_video_test usbcamera_detect_test
                handeye_test minimum_vision_system gimbal_test
                planner_test planner_test_offline)
      add_executable(${tgt} tests/${tgt}.cpp)
      target_link_libraries(${tgt} ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
    endforeach()

    add_executable(auto_aim_debug_solver src/auto_aim_debug_solver.cpp)
    target_link_libraries(auto_aim_debug_solver ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

    add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
    target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

    if(BUILD_SIM)
      add_executable(shm_test tests/shm_test.cpp)
      target_link_libraries(shm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
    endif()
  endif()

  # ---- 需要 auto_buff ----
  if(BUILD_AUTO_BUFF)
    add_executable(auto_buff_test tests/auto_buff_test.cpp)
    target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
  endif()

  # ---- 需要 ROS2 ----
  if(BUILD_NAV)
    foreach(tgt publish_test subscribe_test topic_loop_test)
      add_executable(${tgt} tests/${tgt}.cpp)
      target_link_libraries(${tgt} ${OpenCV_LIBS} fmt::fmt tools io)
    endforeach()
  endif()
endif()

# ============================================================================
# ROS2 程序需要能找到 rm_interfaces 的动态库（dlopen 加载 typesupport 时）
# ============================================================================
if(BUILD_NAV)
  function(sp_link_ros2_rpath tgt)
    if(TARGET ${tgt})
      target_link_options(${tgt} PRIVATE "-Wl,-rpath,${CMAKE_SOURCE_DIR}/rm_interfaces/lib")
      target_link_options(${tgt} PRIVATE "-Wl,-rpath,${CMAKE_SOURCE_DIR}/sp_msgs_install/sp_msgs/lib")
      target_link_options(${tgt} PRIVATE "-Wl,--disable-new-dtags")
    endif()
  endfunction()

  foreach(tgt publish_test subscribe_test topic_loop_test
              sentry sentry_bp sentry_debug sentry_multithread
              standard standard_mpc mt_auto_aim_debug
              auto_aim_sim auto_aim_sim_shm)
    sp_link_ros2_rpath(${tgt})
  endforeach()
endif()

# ============================================================================
# 配置摘要
# ============================================================================
message(STATUS "")
message(STATUS "==================== 构建配置 ====================")
message(STATUS "  自瞄 BUILD_AUTO_AIM          : ${BUILD_AUTO_AIM}")
message(STATUS "  打符 BUILD_AUTO_BUFF         : ${BUILD_AUTO_BUFF}")
message(STATUS "  全向感知 BUILD_OMNIPERCEPTION: ${BUILD_OMNIPERCEPTION}")
message(STATUS "  ROS2 BUILD_NAV               : ${BUILD_NAV}")
message(STATUS "  仿真 BUILD_SIM               : ${BUILD_SIM}")
message(STATUS "  调试 BUILD_DEBUG             : ${BUILD_DEBUG}")
message(STATUS "  测试 BUILD_TESTS             : ${BUILD_TESTS}")
message(STATUS "  标定 BUILD_CALIBRATION       : ${BUILD_CALIBRATION}")
message(STATUS "=================================================")
message(STATUS "")
