cmake_minimum_required(VERSION 3.6)
project(robotics)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
add_executable(testTraj test/test_traj_generator.cpp)
target_include_directories(testTraj PUBLIC inc)
target_link_libraries(testTraj ${GTEST_LIBRARIES} pthread)

add_executable(testRobMat test/test_rob_mat.cpp)
target_include_directories(testRobMat PUBLIC inc)
target_link_libraries(testRobMat ${GTEST_LIBRARIES} pthread)

add_executable(testRobot test/test_robot.cpp)
target_include_directories(testRobot PUBLIC inc)
target_link_libraries(testRobot ${GTEST_LIBRARIES} pthread)

add_executable(robot src/robot.cpp)
target_include_directories(robot PUBLIC inc)
