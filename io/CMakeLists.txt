cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# 串口库（第三方，自带 CMakeLists）
add_subdirectory(serial)

# ============================================================================
# io 拆成多个 target，按需链接：
#
#   io            聚合 target（INTERFACE），让 target_link_libraries(xxx ... io) 继续可用
#   io_core       相机抽象 / C 板 / 云台
#   io_hikrobot   海康相机后端      （IO_HIKROBOT）
#   io_mindvision 迈德威视相机后端   （IO_MINDVISION）
#   io_usbcam     USB 相机
#   io_imu        DM IMU + XRobot IMU
#   io_shm        共享内存 IPC（BUILD_SIM，需要 C++23）
#   io_ros2       ROS2 桥接 + 仿真 IMU（BUILD_NAV，仿真部分需同时开 BUILD_SIM）
#
# 注意：SDK 的 include / link 目录一律用绝对路径。相对路径放进 target 的
# INTERFACE 属性后在消费端解析不可靠，会表现为链接时找不到 -lxxx。
# ============================================================================

# ---------------------------------------------------------------------------
# 相机后端
# ---------------------------------------------------------------------------
if(IO_HIKROBOT)
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
endif()

if(IO_MINDVISION)
  add_library(io_mindvision STATIC mindvision/mindvision.cpp)
  target_include_directories(io_mindvision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(io_mindvision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/amd64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(io_mindvision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/arm64)
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
  endif()
  target_link_libraries(io_mindvision PUBLIC MVSDK usb-1.0 ${OpenCV_LIBS})
endif()

# ---------------------------------------------------------------------------
# USB 相机 / IMU
# ---------------------------------------------------------------------------
add_library(io_usbcam STATIC usbcamera/usbcamera.cpp)
target_link_libraries(io_usbcam PUBLIC ${OpenCV_LIBS} yaml-cpp)

add_library(io_imu STATIC dm_imu/dm_imu.cpp xrobot_imu/xrobot_imu.cpp)
target_link_libraries(io_imu PUBLIC yaml-cpp serial)

# ---------------------------------------------------------------------------
# 相机抽象 / C 板 / 云台
#
# camera.cpp 由 -DIO_ENABLE_xxx 决定把哪个后端编进去；
# 未启用的后端即使 YAML 里写了也只会抛异常，且不会进链接行。
#
# 这里用 PUBLIC 而不是 PRIVATE：PRIVATE 依赖会被包成 $<LINK_ONLY:...>，
# 链接库能传下去但 SDK 的链接目录传不下去，最终链接会找不到 -lMvCameraControl。
# ---------------------------------------------------------------------------
add_library(io_core STATIC camera.cpp cboard.cpp gimbal/gimbal.cpp)
target_link_libraries(io_core PUBLIC yaml-cpp serial usb-1.0 pthread ${OpenCV_LIBS})

if(IO_HIKROBOT)
  target_link_libraries(io_core PUBLIC io_hikrobot)
  target_compile_definitions(io_core PRIVATE IO_ENABLE_HIKROBOT)
endif()
if(IO_MINDVISION)
  target_link_libraries(io_core PUBLIC io_mindvision)
  target_compile_definitions(io_core PRIVATE IO_ENABLE_MINDVISION)
endif()

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
  add_library(io_ros2 STATIC ros2/publish2sim.cpp ros2/subscribe2sim.cpp)
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
target_link_libraries(io INTERFACE io_core io_usbcam io_imu)
if(BUILD_SIM)
  target_link_libraries(io INTERFACE io_shm)
endif()
if(BUILD_NAV)
  target_link_libraries(io INTERFACE io_ros2)
endif()
