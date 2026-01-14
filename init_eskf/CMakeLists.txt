cmake_minimum_required(VERSION 2.8.3)
project(init_eskf)

# 如果没有指定构建类型，默认为Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wno-unused-parameter")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf
)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES init_eskf_lib
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf
)

###########
## Build ##
###########

# 工具函数库
add_library(init_eskf_utility
    src/utility/utility.cpp
)
target_link_libraries(init_eskf_utility ${catkin_LIBRARIES})
target_include_directories(init_eskf_utility PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Init-ESKF 核心库
add_library(init_eskf_lib
        src/euroc_parser.cpp
)
target_link_libraries(init_eskf_lib 
    init_eskf_utility
    ${catkin_LIBRARIES}
)
target_include_directories(init_eskf_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# EuRoC 解析示例
add_executable(euroc_parser_example 
    src/euroc_example.cpp
)
target_link_libraries(euroc_parser_example 
    init_eskf_lib
    ${catkin_LIBRARIES}
)

# EuRoC 测试脚本
add_executable(euroc_test 
    src/euroc_test.cpp
)
target_link_libraries(euroc_test 
    init_eskf_lib
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS init_eskf_lib init_eskf_utility
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
