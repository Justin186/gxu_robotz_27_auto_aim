cmake_minimum_required(VERSION 3.16)

# OpenVINO 的查找与路径由顶层 CMakeLists 统一处理，这里不再写死
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)