cmake_minimum_required(VERSION 2.8)
project (LEGO)

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

include_directories(${INCLUDE_DIRS})
add_executable( findcircles houghcircle.cpp )

target_link_libraries( findcircles ${OpenCV_LIBS} )
