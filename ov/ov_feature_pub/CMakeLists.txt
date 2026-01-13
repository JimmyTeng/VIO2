cmake_minimum_required(VERSION 2.8.3)
project(ov_feature_pub)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    ov_core
    )

find_package(OpenCV REQUIRED)

catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge ov_core
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

# Add ov_core source directory to include path
# ov_core exports headers from src/ directory
get_filename_component(OV_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ov_core" ABSOLUTE)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OV_CORE_DIR}/src
)

add_executable(ov_feature_pub
    src/ov_feature_pub_node.cpp
    )

target_link_libraries(ov_feature_pub 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    )
