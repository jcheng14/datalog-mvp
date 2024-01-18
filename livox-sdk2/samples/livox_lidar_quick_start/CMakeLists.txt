cmake_minimum_required(VERSION 3.0)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(DEMO_NAME livox_lidar_quick_start)
add_executable(${DEMO_NAME} main_pcd.cpp pcd_file.cpp)

target_link_libraries(${DEMO_NAME}
        PUBLIC
        livox_lidar_sdk_static
        ${PCL_LIBRARIES}
				)
