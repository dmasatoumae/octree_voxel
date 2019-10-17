cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(make_cloud)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

linK_directories(${PCl_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(make_cloud main.cpp)

target_link_libraries(make_cloud ${PCL_LIBRARIES})

