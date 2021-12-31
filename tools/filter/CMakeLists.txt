cmake_minimum_required(VERSION 3.5)
project(filter)
set(CMAKE_CXX_STANDARD 14)
add_definitions("-Wall -g -O0")

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
include_directories("/usr/local/include")
include_directories("../..")

add_executable(filter filter.cpp)

target_link_libraries(filter ${PNG_LIBRARY})