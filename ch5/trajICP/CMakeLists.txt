cmake_minimum_required(VERSION 3.0)

project(trajICP)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")

include_directories("/usr/include/eigen3")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(trajICP trajICP.cpp)
target_link_libraries(trajICP Sophus::Sophus)
target_link_Libraries(trajICP pango_display pango_python)
