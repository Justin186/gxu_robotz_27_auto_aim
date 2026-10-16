cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# 串口库（第三方，自带 CMakeLists）
add_subdirectory(serial)

# ============================================================================
# io 拆成多个 target，按需链接：
#
#   io          聚合 target（INTERFACE），让 target_link_libraries(xxx ... io) 继续可用
#   io_core     相机抽象 / C 板 / 云台
#   io_hikrobot 海康相机后端
#   io_usbcam   USB 相机
#   io_shm      共享内存 IPC（BUILD_SIM，需要 C++23）
#   io_ros2     ROS2 桥接 + 仿真 IMU（BUILD_NAV）
#
# 注意：SDK 的 include / link 目录一律用绝对路径。相对路径放进 target 的
# INTERFACE 属性后在消费端解析不可靠，会表现为链接时找不到 -lxxx。
# ============================================================================

# ---------------------------------------------------------------------------
# 海康相机后端
# ---------------------------------------------------------------------------
add_library(io_hikrobot STATIC hikrobot/hikrobot.cpp)
target_include_directories(io_hikrobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io_hikrobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io_hikrobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()
# usb-1.0：hikrobot 的封装里直接调 libusb，必须由本 target 声明，
# 否则静态库链接顺序会把它排在 -lusb-1.0 之后，导致符号找不到
target_link_libraries(io_hikrobot PUBLIC MvCameraControl usb-1.0 ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
# USB 相机
# ---------------------------------------------------------------------------
add_library(io_usbcam STATIC usbcamera/usbcamera.cpp)
target_link_libraries(io_usbcam PUBLIC ${OpenCV_LIBS} yaml-cpp)

# ---------------------------------------------------------------------------
# 相机抽象 / C 板 / 云台
# ---------------------------------------------------------------------------
# 云台串口协议：由顶层 GIMBAL_PROTOCOL 开关决定编入哪个实现。
# 定义必须是 PUBLIC —— io/gimbal/gimbal.hpp 会把协议头带进所有包含它的翻译单元，
# 只在 PRIVATE 定义的话消费方会因为找不到宏而编译失败。
if(GIMBAL_PROTOCOL STREQUAL "standard")
  set(_gimbal_proto_src gimbal/protocols/standard.cpp)
  set(_gimbal_proto_def GIMBAL_PROTOCOL_STANDARD)
elseif(GIMBAL_PROTOCOL STREQUAL "sentry")
  # 哨兵协议的视觉帧复用标准协议，所以两个实现都要编进来
  set(_gimbal_proto_src gimbal/protocols/standard.cpp gimbal/protocols/sentry.cpp)
  set(_gimbal_proto_def GIMBAL_PROTOCOL_SENTRY)
else()
  message(FATAL_ERROR "未知的云台协议 '${GIMBAL_PROTOCOL}'（可选：standard sentry）")
endif()

add_library(io_core STATIC camera.cpp cboard.cpp gimbal/gimbal.cpp ${_gimbal_proto_src})
target_compile_definitions(io_core PUBLIC ${_gimbal_proto_def})
target_link_libraries(io_core PUBLIC io_hikrobot yaml-cpp serial usb-1.0 pthread ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
# 共享内存 IPC —— 需要 C++23 的 <expected>
# ---------------------------------------------------------------------------
if(BUILD_SIM)
  add_library(io_shm STATIC shm_ipc/shm_adapter.cpp)
  target_include_directories(io_shm PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/shm_ipc)
  target_link_libraries(io_shm PUBLIC yaml-cpp ${OpenCV_LIBS})

  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-std=c++23" COMPILER_SUPPORTS_CXX23)
  if(COMPILER_SUPPORTS_CXX23)
    target_compile_options(io_shm PRIVATE -std=c++23)
    message(STATUS "共享内存 IPC: 使用 C++23")
  else()
    message(WARNING "C++23 not supported, shared memory IPC may not compile")
  endif()
endif()

# ---------------------------------------------------------------------------
# ROS2 桥接（含依赖 ROS2 的仿真 IMU）
# ---------------------------------------------------------------------------
if(BUILD_NAV)
  add_library(io_ros2 STATIC ros2/publish2sim.cpp ros2/subscribe2sim.cpp ros2/ros2.cpp)
  target_include_directories(io_ros2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  # sim_imu 内部直接用了 rclcpp / tf2，所以它属于 ROS2 这一组，
  # 只有 BUILD_SIM 时才有存在的意义
  if(BUILD_SIM)
    target_sources(io_ros2 PRIVATE sim_imu/sim_imu.cpp)
  endif()

  ament_target_dependencies(io_ros2
      rclcpp
      std_msgs
      sensor_msgs
      cv_bridge
      rm_interfaces
      sp_msgs
      geometry_msgs
      tf2_ros
      tf2_geometry_msgs
      tf2
  )
endif()

# ---------------------------------------------------------------------------
# 聚合 target：保持既有写法 target_link_libraries(xxx ... io) 可用
# ---------------------------------------------------------------------------
add_library(io INTERFACE)
target_link_libraries(io INTERFACE io_core io_usbcam)
if(BUILD_SIM)
  target_link_libraries(io INTERFACE io_shm)
endif()
if(BUILD_NAV)
  target_link_libraries(io INTERFACE io_ros2)
endif()
