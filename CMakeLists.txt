cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(flvc)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    message("Building voxel-io DEBUG")
    add_definitions(-DVXIO_DEBUG)
else()
    message("Building voxel-io RELEASE")
    add_definitions(-DVXIO_RELEASE)
endif()

add_subdirectory(voxelio)
include_directories(voxelio/src)

set(SOURCES main.cpp flvc.hpp flvc.cpp svo.hpp svo_nodes.hpp)

add_executable(flvc "${SOURCES}")
target_include_directories(flvc PUBLIC voxelio/src/)
target_link_libraries(flvc voxelio)
