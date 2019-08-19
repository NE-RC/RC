cmake_minimum_required(VERSION 2.8.3)
project(RC)

add_compile_options(-std=c++11) # -pedantic -Wall -Werror -Wfatal-errors -Wextra -Wno-unused-parameter -Wno-unused-variable)

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

add_library(pid src/pid.cpp)
add_library(ros_pid src/ros_pid.cpp)

add_executable(velocity_controller_node src/velocity_controller_node.cpp)
add_executable(encoder src/encoder.cpp)

target_link_libraries(velocity_controller_node ${catkin_LIBRARIES} ros_pid pid)
target_link_libraries(encoder ${catkin_LIBRARIES})
