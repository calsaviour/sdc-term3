project(BehaviourPlanning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/road.cpp
    src/vehicle.cpp
    src/main.cpp)

add_executable(BehaviourPlanning ${sources})