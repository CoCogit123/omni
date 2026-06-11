cmake_minimum_required(VERSION 3.10)
project(omni_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MuJoCo
set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/mujoco/mujoco_3_1")
find_library(MUJOCO_LIB mujoco PATHS ${MUJOCO_DIR}/lib REQUIRED)

# Find mujoco_interface libraries
set(MUJOCO_INTERFACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/mujoco_interface")
set(MUJOCO_INTERFACE_BUILD_DIR "${MUJOCO_INTERFACE_DIR}/build")

# Find plot_interface libraries
set(PLOTJUGGLER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/plotjuggler_interface")
set(PLOTJUGGLER_BUILD_DIR "${PLOTJUGGLER_DIR}/build")

# Include directories
include_directories(
    ${MUJOCO_DIR}/include
    ${MUJOCO_INTERFACE_DIR}/include
    ${PLOTJUGGLER_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/include/eigen3
)

# Link directories
link_directories(
    ${MUJOCO_DIR}/lib
    ${MUJOCO_INTERFACE_BUILD_DIR}
    ${PLOTJUGGLER_BUILD_DIR}
)

# Add executable
add_executable(omni_sim
    simple.cpp
    src/ballbot_pid.cpp
)

# Link libraries
target_link_libraries(omni_sim
    ${MUJOCO_LIB}
    ${MUJOCO_INTERFACE_BUILD_DIR}/libui_interface.a
    ${MUJOCO_INTERFACE_BUILD_DIR}/libmj_interface.a
    ${PLOTJUGGLER_BUILD_DIR}/libplotjuggler_interface.a
    glfw
    GL
    pthread
    dl
    yaml-cpp
)

# Set output directory
set_target_properties(omni_sim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

message(STATUS "MuJoCo directory: ${MUJOCO_DIR}")
message(STATUS "MuJoCo library: ${MUJOCO_LIB}")
message(STATUS "MuJoCo Interface directory: ${MUJOCO_INTERFACE_DIR}")
