cmake_minimum_required(VERSION 2.8.3)
project(RC)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

add_message_files(
    FILES
    pwmout.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(
    pid src/pid.cpp
)
