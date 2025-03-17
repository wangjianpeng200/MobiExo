cmake_minimum_required(VERSION 2.8.3)
project(road_lane_publisher)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp 
        compute
        custom_msgs
        geometry_msgs
        )
find_package(Eigen3 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_map_publisher
#  CATKIN_DEPENDS compute custom_msgs pcl_ros roscpp sensor_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/road_lane.cpp
                        )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})        
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
  -lpqxx
  -lpq
  )