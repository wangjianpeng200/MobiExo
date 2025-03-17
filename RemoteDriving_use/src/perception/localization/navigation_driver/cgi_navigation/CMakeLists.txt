cmake_minimum_required(VERSION 2.8.3)
project(cgi_navigation)

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

add_executable(cgi_navigation_node
   src/main.cpp
   src/uart_api.cpp)
add_dependencies(cgi_navigation_node ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(cgi_navigation_node
 ${catkin_LIBRARIES}
)

