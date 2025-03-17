cmake_minimum_required(VERSION 2.8.3)
project(gnss_localizer)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  compute
  custom_msgs
  roscpp
  tf
  visualization_msgs
)

find_package(
    Eigen3 REQUIRED
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
#  CATKIN_DEPENDS compute custom_msgs roscpp tf 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}

)


 add_executable(simple_localization_node src/simple_localization.cpp)
 add_executable(simple_localization_slam_gnss_node src/simple_localization_slam_gnss.cpp)
 add_executable(simple_localization_uwb_node src/simple_localization_uwb.cpp)
 add_executable(simple_localization_slamindoor_node src/simple_localization_slamindoor.cpp)

 add_dependencies(simple_localization_node
     ${custom_msgs_EXPORTED_TARGETS}
     ${compute_EXPORTED_TARGETS})

 add_dependencies(simple_localization_slam_gnss_node
     ${custom_msgs_EXPORTED_TARGETS}
     ${compute_EXPORTED_TARGETS}  
     )

 add_dependencies(simple_localization_uwb_node 
     ${custom_msgs_EXPORTED_TARGETS}
     ${compute_EXPORTED_TARGETS})

 add_dependencies(simple_localization_slamindoor_node
     ${custom_msgs_EXPORTED_TARGETS}
     ${compute_EXPORTED_TARGETS})


 target_link_libraries(simple_localization_node
   ${catkin_LIBRARIES})

 target_link_libraries(simple_localization_slam_gnss_node
   ${catkin_LIBRARIES})

 target_link_libraries(simple_localization_uwb_node
   ${catkin_LIBRARIES})

 target_link_libraries(simple_localization_slamindoor_node
   ${catkin_LIBRARIES})





 
