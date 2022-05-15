cmake_minimum_required(VERSION 3.0.2)
project(2d_trajectory_generation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  trajectory_msgs
  std_msgs
  message_filters
  costmap_2d
  tf2_ros
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system)
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
#find_package(GUROBI REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generation
  CATKIN_DEPENDS 
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    trajectory_msgs
    std_msgs
    costmap_2d
    tf2_ros
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/trajectory_generation
  ${catkin_INCLUDE_DIRS}
  #${GUROBI_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})


#add_executable(${PROJECT_NAME}_node   
#  src/trajectory_generation_node.cpp
#  src/PolynomialTrajectoryGeneration.cpp
#)

#target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   optimized ${GUROBI_CXX_LIBRARY} 
#   debug ${GUROBI_CXX_DEBUG_LIBRARY}
#   ${GUROBI_LIBRARY}
#   Eigen3::Eigen
#)

add_executable(${PROJECT_NAME}_node
  src/trajectory_generation_node.cpp
  #src/PolynomialTrajectoryGeneration.cpp
  src/TrajectoryGenerationOptimalControl.cpp
  src/RayGeneration.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   #optimized ${GUROBI_CXX_LIBRARY} 
   #debug ${GUROBI_CXX_DEBUG_LIBRARY}
   #${GUROBI_LIBRARY}
   Eigen3::Eigen
)

add_executable(controller_node
  src/controller_node.cpp
)


target_link_libraries(controller_node
   ${catkin_LIBRARIES}
   Eigen3::Eigen
)
