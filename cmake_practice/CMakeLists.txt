# CMAKE version
cmake_minimum_required(VERSION 3.0)

# Project Name
project(cmake_practice)

# Assign Release to Compile Modes CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE "Release")

# Compile Executable
add_executable(sayHello useHello.cpp)

# Add the head file = ~/include/hello.h
# If we use find_package() function, the system will search /usr/local/include fold
include_directories(${PROJECT_SOURCE_DIR}/include)

# Compile Dynamic Lib
add_library(hello SHARED ${PROJECT_SOURCE_DIR}/src/hello.cpp)

# create the link to connect libraries with the main code
target_link_libraries(sayHello hello)

# put hello.h into /usr/local/include
install(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION /usr/local/include)

# put hello.so into /usr/local/lib
install(TARGETS hello LIBRARY DESTINATION /usr/local/lib)
