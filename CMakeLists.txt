cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)

## Compile as C++11, supported in ROS Kinetic and newer
SET( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
add_compile_options(-std=c++17)
add_definitions(-DPROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=4)
  

find_package(OpenMP QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  CloudWithPose.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)



find_package(PCL REQUIRED)

## SC
find_package(OpenCV REQUIRED)
# YAML CPP
find_package(PkgConfig REQUIRED)
find_package(Ceres REQUIRED)


pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieskf_slam
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)
include_directories(
 include
 lib
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
)
SET(IESKF_SLAM_FILE
    src/ieskf_slam/modules/frondend/frontend.cpp
    src/ieskf_slam/modules/backend/backend.cpp
    src/ieskf_slam/modules/ieskf/ieskf.cpp
    src/ieskf_slam/modules/map/rect_map_manager.cpp
    src/ieskf_slam/modules/frontback_propagate/frontback_propagate.cpp
    src/ieskf_slam/modules/pose_graph_opt/pose_graph_opt.cpp

)

SET(LIB_FILE
lib/scan_context/Scancontext.cpp
)
SET(NOETIC_FRONTEND_WRAPPER_FILE
    src/wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.cpp
)
SET(NOETIC_BACKEND_WRAPPER_FILE
    src/wrapper/ros_noetic/ieskf_backend_noetic_wrapper.cpp
)

add_executable(ieskf_frontend_node src/app/ros_noetic/ieskf_noetic_running_node.cpp
  ${NOETIC_FRONTEND_WRAPPER_FILE}
  ${IESKF_SLAM_FILE}
  ${LIB_FILE}
)

target_link_libraries(ieskf_frontend_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Ceres::ceres
)


add_executable(ieskf_backend_node src/app/ros_noetic/ieskf_noetic_backend_node.cpp
${NOETIC_BACKEND_WRAPPER_FILE}
${IESKF_SLAM_FILE}
${LIB_FILE}
)

target_link_libraries(ieskf_backend_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Ceres::ceres
)
