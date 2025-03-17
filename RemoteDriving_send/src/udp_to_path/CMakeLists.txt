cmake_minimum_required(VERSION 3.0.2)
project(udp_to_path)

## 添加 C++ 标准
set(CMAKE_CXX_STANDARD 14)

## 查找 catkin 依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

## 声明 catkin 工程
catkin_package()

## 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 添加可执行文件并链接库
add_executable(udp_to_path src/udp_to_path.cpp)
target_link_libraries(udp_to_path ${catkin_LIBRARIES})
