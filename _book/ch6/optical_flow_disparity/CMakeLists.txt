cmake_minimum_required(VERSION 3.0)

project(opticalFlow)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(SRC_LIST ${PROJECT_SOURCE_DIR}/src/optical_flow_disparity.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories("/usr/include/eigen3")

find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(opticalFlow ${SRC_LIST})
target_link_libraries(opticalFlow ${OpenCV_LIBRARIES})
target_link_libraries(opticalFlow Sophus::Sophus)
target_link_libraries(opticalFlow pango_display pango_python)
