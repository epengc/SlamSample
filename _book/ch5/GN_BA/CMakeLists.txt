cmake_minimum_required(VERSION 3.0)

project(GN_BA)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories("/usr/include/eigen3")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(GN_BA GN-BA.cpp)
target_link_libraries(GN_BA Sophus::Sophus)


