cmake_minimum_required(VERSION 3.0.2)

project(OptimHW_1)

include_directories("/usr/include/eigen3")

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node main.cpp)