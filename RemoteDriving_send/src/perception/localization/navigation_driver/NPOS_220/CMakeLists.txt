cmake_minimum_required(VERSION 2.8.3)
project(npos_220)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
   roscpp
   custom_msgs
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

add_executable(${PROJECT_NAME}_node
   src/main.cpp
   src/serial_port_interface.cpp)
add_dependencies(${PROJECT_NAME}_node ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
)
