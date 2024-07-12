cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(FlexibleStickyPlugin SHARED FlexibleStickyPlugin.cpp)
target_link_libraries(FlexibleStickyPlugin ${GAZEBO_LIBRARIES})
