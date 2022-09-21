cmake_minimum_required(VERSION 3.10)

# set the project name
project(csc4140_a1)

find_package(OpenCV REQUIRED)

# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})

# add the executable
add_executable(csc4140_a1_p2_1 p2_1.cpp)
add_executable(csc4140_a1_p2_2 p2_2.cpp)
add_executable(csc4140_a1_p2_3 p2_3.cpp)
add_executable(csc4140_a1_p2_4 p2_4.cpp)

# Link your application with OpenCV libraries
target_link_libraries(csc4140_a1_p2_3 ${OpenCV_LIBS})