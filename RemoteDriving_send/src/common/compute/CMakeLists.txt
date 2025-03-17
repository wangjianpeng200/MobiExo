cmake_minimum_required(VERSION 2.8.3)
project(compute)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES commonfunc trans_crood map frenet
)

include_directories(
 include
 include/compute
)

# Declare a C++ library
 add_library(commonfunc STATIC
   src/CommonFuction.cpp
 )

# Declare a C++ library
 add_library(trans_crood STATIC
   src/trans_coord.cpp
 )

 # Declare a C++ library
 add_library(map STATIC
   src/map.cpp
 )

 # Declare a C++ library
 add_library(frenet STATIC
   src/frenet.cpp
 )

include_directories( ${EIGEN3_INCLUDE_DIR})

