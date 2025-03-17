cmake_minimum_required(VERSION 2.8.3)
project(nova_nav)

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

add_executable(node_nova_navigation
   src/novatel_gps_imu.cpp
   src/node_navigation.cpp)
add_dependencies(node_nova_navigation ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(node_nova_navigation
 ${catkin_LIBRARIES}
)


add_executable(cvtgeopoint
   src/cvtgeopoint.cpp)
add_dependencies(cvtgeopoint ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(cvtgeopoint
 ${catkin_LIBRARIES}
)


add_executable(save_txt
   src/save_txt.cpp)
add_dependencies(save_txt ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(save_txt
 ${catkin_LIBRARIES}
)
