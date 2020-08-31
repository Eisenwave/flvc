cmake_minimum_required(VERSION 3.2)
project(flvc)

add_subdirectory(voxelio)

set(SOURCES main.cpp)

add_executable(flvc "${SOURCES}")
target_link_libraries(flvc voxelio)
