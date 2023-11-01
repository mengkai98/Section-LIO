cmake_minimum_required(VERSION 3.0.2)
project(section_lio)
SET(CMAKE_BUILD_TYPE "Rlease")
SET( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions(-DSLAM_CRAFT_PROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 10)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=10)
    message("core for MP: 10")
  elseif(N GREATER 4 AND N LESS 10)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=(${N}-1))
    message("core for MP: 10")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  livox_ros_driver
  roscpp
  rospy
  std_msgs
  cv_bridge
)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slam_craft_turbo
#  CATKIN_DEPENDS livox_ros_driver roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
  include/libs/ikd-Tree
)

add_executable(slio_node src/slio_node.cpp )
target_link_libraries(slio_node
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${YAML_CPP_LIBRARIES}
glog::glog
)