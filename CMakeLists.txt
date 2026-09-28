cmake_minimum_required(VERSION 3.16)
project(filters LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(filters
    src/filters/base.cpp
    src/filters/ekf.cpp
    src/filters/lkf.cpp
)

target_include_directories(filters PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(filters PUBLIC Eigen3::Eigen)

add_executable(lkf_const_vel src/lkf_const_vel_3d.cpp)
target_link_libraries(lkf_const_vel PRIVATE filters)
