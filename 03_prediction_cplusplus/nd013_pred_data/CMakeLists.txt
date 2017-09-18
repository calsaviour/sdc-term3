project(Classifier)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/classifier.cpp
    src/main.cpp)

add_executable(Classifier ${sources})