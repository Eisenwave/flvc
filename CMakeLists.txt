cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(flvc)

string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)

if(build_type STREQUAL DEBUG)
    message("Building FLVC DEBUG")
    add_definitions(-DVXIO_DEBUG)
elseif(build_type STREQUAL RELWITHDEBINFO)
    message("Building FLVC RELEASE (with debug info)")
    add_definitions(-DVXIO_RELEASE)
else()
    message("Building FLVC RELEASE")
    add_definitions(-DVXIO_RELEASE)
endif()

add_subdirectory(voxelio)
include_directories(voxelio/src)

set(SOURCES
    main.cpp
    flvcattrib.hpp
    flvccodec.hpp
    flvccodec.cpp
    flvcconfig.hpp
    flvcconst.hpp
    permutation.hpp
    svo.hpp)

add_executable(flvc "${SOURCES}")
target_include_directories(flvc PUBLIC voxelio/src/)
target_link_libraries(flvc voxelio)
