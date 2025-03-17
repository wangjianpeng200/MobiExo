cmake_minimum_required(VERSION 2.8.3)
project(dtu)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
   roscpp
   custom_msgs
   geographic_msgs
   sensor_msgs
   tf
   )

include_directories(
   ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oxts_imu
#  CATKIN_DEPENDS roscpp
#  DEPENDS
)

add_executable(dtu_node
   src/main.cpp
   src/dtu_api.cpp
   src/socket_api.cpp
   src/uart_api.cpp)
add_dependencies(dtu_node ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(dtu_node
 ${catkin_LIBRARIES}
)

