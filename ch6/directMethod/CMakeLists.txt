cmake_minimum_required( VERSION 3.0 )
SET(CMAKE_BUILD_TYPE "Release")

PROJECT(DirectMethod)
SET(CMAKE_CXX_FLAGS "-std=c++14")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES("/usr/include/opencv2")
INCLUDE_DIRECTORIES("/usr/include/eigen3")

find_package( OpenCV REQUIRED )

find_package( Sophus REQUIRED )
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

SET(SRC_LIST ${PROJECT_SOURCE_DIR}/src/direct_method.cpp)
ADD_EXECUTABLE(direct_method ${PROJECT_SOURCE_DIR}/src/direct_method.cpp)
target_link_libraries(direct_method Sophus::Sophus)
target_link_libraries(direct_method ${Pangolin_LIBRARIES} ${OpenCV_LIBRARIES})
