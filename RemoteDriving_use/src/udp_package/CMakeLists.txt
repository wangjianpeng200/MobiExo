cmake_minimum_required(VERSION 3.0.2)
project(udp_package)

## 找到 catkin 和其他依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  custom_msgs

)

## 编译源文件
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(udp_sender src/udp_sender.cpp)

target_link_libraries(udp_sender
  ${catkin_LIBRARIES}
)
