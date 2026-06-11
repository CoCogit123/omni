cmake_minimum_required(VERSION 3.10)
project(mujoco_interface)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找工作空间路径
get_filename_component(WORKSPACE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

# 设置 MuJoCo 和 GLFW 路径
set(MUJOCO_DIR ${WORKSPACE_PATH}/mujoco/mujoco_3_1)
set(GLFW_DIR ${WORKSPACE_PATH}/mujoco/glfw3)

# 检查 MuJoCo 是否存在
if(NOT EXISTS ${MUJOCO_DIR})
    message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_DIR}")
endif()

if(NOT EXISTS ${GLFW_DIR})
    message(FATAL_ERROR "GLFW not found at ${GLFW_DIR}")
endif()

# 查找 Eigen3
find_package(Eigen3 REQUIRED)

# 查找 yaml-cpp (用于解析 imgui.yaml)
find_package(yaml-cpp REQUIRED)

# Dear ImGui 源码路径
set(IMGUI_DIR ${WORKSPACE_PATH}/dear_imgui)

# 包含头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_DIR}/include
    ${GLFW_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${IMGUI_DIR}
)

# 链接库目录
link_directories(
    ${MUJOCO_DIR}/lib
    ${GLFW_DIR}/lib
)

# 添加 UI_interface 库
add_library(ui_interface STATIC
    src/UI_interface.cpp
    src/tunable_param_registry.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/imgui_impl_opengl3.cpp
)

target_link_libraries(ui_interface
    ${MUJOCO_DIR}/lib/libmujoco.so
    ${GLFW_DIR}/lib/libglfw3.a
    yaml-cpp::yaml-cpp
    -lX11 -lXrandr -lXi -lXxf86vm -lXinerama -lpthread -lrt -ldl -lXcursor
)

# 添加 MJ_interface 库
add_library(mj_interface STATIC
    src/MJ_interface.cpp
)

target_link_libraries(mj_interface
    ${MUJOCO_DIR}/lib/libmujoco.so
)
