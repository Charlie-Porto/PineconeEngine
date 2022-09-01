cmake_minimum_required(VERSION 3.10)

project(PCE_Example1)

add_executable(PCE_Example1 main.cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(pce_core STATIC ../../include/pceSDL/pceSDL.cpp)
add_library(pce_ecs STATIC ../../include/pcecs/pcecs.cpp)
add_library(pce_3d STATIC ../../include/pce3d/pce3d.cpp)
add_library(pce_3d_modules STATIC ../../include/pce3d/modules.cpp)
target_link_libraries(PCE_Example1 PUBLIC pce_core pce_ecs pce_3d pce_3d_modules)



