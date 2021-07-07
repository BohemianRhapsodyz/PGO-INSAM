cmake_minimum_required(VERSION 3.0.2)
project(stim300)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-o3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        tf
        rosbag
        pcl_conversions
        )

find_package(cmake_modules REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
#find_package(Eigen3 REQUIRED)

#find_package(PCL 1.7 REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES lidar_imu_calib
        #  CATKIN_DEPENDS roscpp rospy sensor_msgs
        #  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN3_DEFINITIONS})
link_directories(
        include
        ${GTSAM_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_library(cpsins
        src/PSINS.cpp
        src/KFApp.cpp
        )

target_link_libraries(cpsins ${catkin_LIBRARIES} )

add_executable(od_sins_node src/main.cpp)
target_link_libraries(od_sins_node cpsins ${PCL_LIBRARIES} gtsam)
