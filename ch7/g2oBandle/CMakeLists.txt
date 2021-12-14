cmake_minimum_required(VERSION 3.0)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS "-std=c++14")
PROJECT(g2o_bundle)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(G2O REQUIRED)
include_directories("/usr/local/include/g2o/EXTERNAL/csparse")

find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIRS})
#find_package(glog REQUIRED)
#include_directories(${glog_INCLUDE_DIRS})

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)

include_directories(${PROJECT_SOURCE_DIR}${EIGEN3_INCLUDE_DIR}${CSPARSE_INCLUDE_DIR})

add_library(bal_common common.cpp)
add_executable(g2o_bundle ${PROJECT_SOURCE_DIR}/src/g2o_bundle.cpp)

target_link_libraries(g2o_bundle ${G2O_LIBS} bal_common)
target_link_libraries(g2o_bundle Sophus::Sophus)
target_link_libraries(g2o_bundle Ceres::ceres)
