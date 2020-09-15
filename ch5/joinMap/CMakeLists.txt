cmake_minimum_required(VERSION 2.8)
project(point_cloud_join)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/eigen3/")
include_directories("/usr/include/pcl-1.8")


find_package(PCL REQUIRED QUIET
 COMPONENTS common io
 )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(joinMap joinMap.cpp)

target_link_libraries(joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES})
