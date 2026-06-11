cmake_minimum_required(VERSION 3.10)
project(plotjuggler_interface)

# C++14 标准 (与 mujoco_interface 保持一致)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 编译为静态库
add_library(plotjuggler_interface STATIC
    src/PlotJuggler_interface.cpp
)

# 无外部依赖 — 仅使用 C++ 标准库和 POSIX socket (libc 自带)
