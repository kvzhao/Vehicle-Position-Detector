cmake_minimum_required(VERSION 2.8)
project (LEGO)

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "inc/*.h")

set (INCLUDE_DIRS "")
foreach (_headerFile ${HEADERS})
        get_filename_component(_dir ${_headerFile} PATH)
        list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

include_directories(${INCLUDE_DIRS})

add_executable( PositionDetector main.cpp ${SOURCES})
target_link_libraries( PositionDetector ${OpenCV_LIBS} )
