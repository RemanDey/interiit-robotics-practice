cmake_minimum_required(VERSION 3.16)
project(ackermann_kinodynamic_sim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Core Library
add_library(ackermann_sim_lib
    src/vehicle_model.cpp
    src/occupancy_grid.cpp
    src/simulator.cpp
)

target_link_libraries(ackermann_sim_lib
    PRIVATE
    ${OpenCV_LIBS}
    pthread
)

# 1. Simulator Server Executable
add_executable(simulator_node apps/simulator_node.cpp)
target_link_libraries(simulator_node PRIVATE ackermann_sim_lib ${OpenCV_LIBS})

# 2. Candidate Starter Assessment Template Executable
add_executable(candidate_template apps/candidate_template.cpp)
target_link_libraries(candidate_template PRIVATE pthread)

message(STATUS "Inter IIT Tech Meet: Kinodynamic Simulator configured successfully.")
