cmake_minimum_required(VERSION 3.10)

# set the project name
project(csc4140_a1)

find_package(OpenCV REQUIRED)

# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB hw2_src *.cpp)

add_library( # Sets the name of the library.
             hw2_lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             ${hw2_src})

# add the executable
add_executable(main ${hw2_src})

# Link your application with OpenCV libraries
target_link_libraries(main ${OpenCV_LIBS})