cmake_minimum_required(VERSION 3.5)
project(RayTracer)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
include_directories("/usr/local/include")
include_directories("..")

add_executable(RayTracer main.cpp Camera.h Canvas.h config.h Object.h Scene.h )

target_link_libraries(RayTracer ${PNG_LIBRARY})