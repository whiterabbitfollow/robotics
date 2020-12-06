cmake_minimum_required(VERSION 3.6)
project(robotics)
add_executable(robot src/robot.cpp)
target_include_directories(robot PUBLIC inc)
