cmake_minimum_required(VERSION 2.8.3)
project(lidarmarker)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp 
        compute
        cv_bridge
        custom_msgs
        visualization_msgs
        )
find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_map_publisher
#  CATKIN_DEPENDS compute custom_msgs pcl_ros roscpp sensor_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories( ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})

add_executable(lidar_marker src/main.cpp
                           src/marker.cpp 
                        )
add_dependencies(lidar_marker ${catkin_EXPORTED_TARGETS}) 
add_dependencies(lidar_marker
     ${custom_msgs_EXPORTED_TARGETS})                          
target_link_libraries(lidar_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  -lpqxx
  -lpq
  )