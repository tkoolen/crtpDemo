cmake_minimum_required(VERSION 2.8.6)
project(crtpDemo)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(Eigen3 3.2.5)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(crtpDemo crtpDemo.cpp)

# workaround to get all headers in project
FILE (GLOB_RECURSE clion_all_headers ${CMAKE_SOURCE_DIR}/*.h)
ADD_CUSTOM_TARGET(all_clion SOURCES ${clion_all_headers})
