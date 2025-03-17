cmake_minimum_required(VERSION 3.0.2)
project(map_switch_client_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  custom_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_switch_client src/map_switch_client.cpp)
target_link_libraries(map_switch_client ${catkin_LIBRARIES})
add_dependencies(map_switch_client custom_msgs_generate_messages_cpp)
