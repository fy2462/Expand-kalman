project(ExtendedKF)
cmake_minimum_required (VERSION 3.5)
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

Find_Package(Eigen3 REQUIRED)
############### source and test ######################
include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/ceres)
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)