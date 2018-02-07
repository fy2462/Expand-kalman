project(ExtendedKF)
cmake_minimum_required (VERSION 3.5)
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

Find_Package(Eigen3 REQUIRED)
############### source and test ######################
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/common)
add_subdirectory(src)
AUX_SOURCE_DIRECTORY(src DIR_SRCS)