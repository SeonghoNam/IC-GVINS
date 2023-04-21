cmake_minimum_required(VERSION 3.10)
project(ic_gvins)

set(CMAKE_CXX_STANDARD 14)

# Optimization
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Build GVINS library
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ic_gvins)
file(GLOB_RECURSE GVINS_SOURCE
        ic_gvins/*.cc
        ic_gvins/common/*.cc
        ic_gvins/fileio/*.cc
        ic_gvins/tracking/*.cc
        ic_gvins/preintegration/*.cc)
add_library(ic_gvins_core SHARED
        ${GVINS_SOURCE})

# OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(ic_gvins_core ${OpenCV_LIBS})

# glog
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
target_link_libraries(ic_gvins_core ${GLOG_LIBRARIES})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(ic_gvins_core ${YAML_CPP_LIBRARIES})

# Ceres
find_package(Ceres REQUIRED)
target_link_libraries(ic_gvins_core ${CERES_LIBRARIES})

# boost
find_package(Boost REQUIRED COMPONENTS filesystem)
target_link_libraries(ic_gvins_core
        ${Boost_LIBRARIES})

# TBB
find_package(TBB REQUIRED)
target_link_libraries(ic_gvins_core
        TBB::tbb)

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
target_link_libraries(ic_gvins_core ${Pangolin_LIBRARIES} GL GLU GLEW glut)

# app
add_executable(run_kitti app/run_kitti.cc)
target_link_libraries(run_kitti ic_gvins_core)

# # abseil
# set(ABSL_PROPAGATE_CXX_STD true)
# add_subdirectory(thirdparty/abseil-cpp abseil-cpp)
# target_link_libraries(ic_gvins_core
#         absl::strings
#         absl::str_format
#         absl::time)

# # ROS interface
# find_package(ament_cmake REQUIRED COMPONENTS
#         message_generation
#         roscpp
#         std_msgs)
# catkin_package(
#         LIBRARIES
#         CATKIN_DEPENDS roscpp std_msgs)
# include_directories(
#         ${catkin_INCLUDE_DIRS})

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}_ros
#   "msg/Imu.msg"
#   "msg/RadarScan.msg"
#   "msg/RadarTrack.msg"
#   "msg/RadarTracks.msg"
#   DEPENDENCIES std_msgs geometry_msgs 
# )

# include_directories(include)        

# # GVINS ROS with standard messages
# add_executable(${PROJECT_NAME}_ros
#         ROS/fusion_ros.cc
#         ROS/drawer_rviz.cc)

# #         add_dependencies(${PROJECT_NAME}_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_ros
#         ic_gvins_core)
# ament_target_dependencies(${PROJECT_NAME}_ros
#   rclcpp
#   std_msgs)


# ament_export_include_directories(include)  
# ament_export_include_directories(include)
# ament_export_dependencies(std_msgs)
# ament_package()