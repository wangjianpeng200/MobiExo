cmake_minimum_required(VERSION 3.0.2)
project(path_publisher)

## 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找 Catkin 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf
)

## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pure_pursuit
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs tf2_ros tf2 tf
)

## 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## 声明 C++ 可执行文件
add_executable(path_publisher src/path_publisher.cpp)

## 将可执行文件与 Catkin 库链接
target_link_libraries(path_publisher
  ${catkin_LIBRARIES}
)

## 如果你有自定义的库，也可以在此添加
# target_link_libraries(pure_pursuit_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
# )
