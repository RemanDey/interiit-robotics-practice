cmake_minimum_required(VERSION 3.16)
project(task2_ros2_can_uart_sim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Core Hardware Abstraction Library
add_library(rover_hardware_lib
    src/can_protocol.cpp
    src/uart_protocol.cpp
    src/rover_hardware_interface.cpp
)

target_link_libraries(rover_hardware_lib
    PRIVATE
    pthread
)

# 1. Hardware Node Test Executable
add_executable(hardware_node apps/hardware_node.cpp)
target_link_libraries(hardware_node PRIVATE rover_hardware_lib pthread)

# 2. Candidate Starter Assessment Template Executable
add_executable(candidate_template apps/candidate_template.cpp)
target_link_libraries(candidate_template PRIVATE pthread)

message(STATUS "Task 2: Custom ros2_control Hardware Interface over CAN and UART configured successfully.")
