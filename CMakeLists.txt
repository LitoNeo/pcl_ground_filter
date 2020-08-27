cmake_minimum_required(VERSION 3.14)
project(ground_filter_shared)

set(CMAKE_CXX_STANDARD 11)

#find_package(catkin REQUIRED COMPONENTS
#        pcl_ros
##        roscpp
##        sensor_msgs
##        geometry_msgs
#        )

find_package(PCL 1.7 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(
        ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(ground_filter ray_ground_filter.cpp config.h config.cpp ground_filter_core.h ground_filter_core.cpp ray_ground_filter.h common.h common.cpp)
#add_library(ground_filter SHARED library.cpp library.h config.h config.cpp ground_filter_core.h ground_filter_core.cpp)
#add_library(ground_filter SHARED ray_ground_filter.cpp config.h config.cpp ground_filter_core.h ground_filter_core.cpp)
#add_library(test SHARED test.cpp)
target_link_libraries(ground_filter
        ${PCL_LIBRARIES}
        )