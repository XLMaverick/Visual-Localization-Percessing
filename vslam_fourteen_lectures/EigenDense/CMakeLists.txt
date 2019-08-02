cmake_minimum_required(VERSION 2.8)
project(homework_1th)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories("/usr/include/eigen3")
add_executable(eigenMatrix main.cpp)