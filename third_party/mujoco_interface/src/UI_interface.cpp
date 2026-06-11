#include "UI_interface.h"
#include "tunable_param_registry.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "yaml-cpp/yaml.h"

#include <cstdio>
#include <cmath>

// 构造函数
UIInterface::UIInterface(mjModel* model, mjData* data)
    : mj_model_(model),
      mj_data_(data),
      window_(nullptr),
      last_device_mouse_x_(0.0),
      last_device_mouse_y_(0.0),
      is_initialized_(false) {

    // 初始化 MuJoCo 可视化对象
    cam_ = mjvCamera();
    opt_ = mjvOption();
    scn_ = mjvScene();
    con_ = mjrContext();
}

// 析构函数
UIInterface::~UIInterface() {
    close();
}

// 设置可调参数注册表
void UIInterface::setTunableParamRegistry(TunableParamRegistry* registry) {
    tunable_registry_ = registry;
}

// 初始化并创建窗口
bool UIInterface::initialize() {
    if (is_initialized_) {
        return true;
    }

    // 初始化 GLFW
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return false;
    }

    // 创建窗口
    window_ = glfwCreateWindow(cfg_window.width, cfg_window.height, cfg_window.title.c_str(), nullptr, nullptr);
    if (!window_) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // 启用垂直同步

    // 设置相机
    mjv_defaultCamera(&cam_);
    cam_.azimuth = cfg_camera.azimuth;
    cam_.elevation = cfg_camera.elevation;
    cam_.distance = cfg_camera.distance;
    cam_.lookat[0] = cfg_camera.lookat_x;
    cam_.lookat[1] = cfg_camera.lookat_y;
    cam_.lookat[2] = cfg_camera.lookat_z;

    if (cfg_camera.enable_tracking) {
        cam_.type = mjCAMERA_TRACKING;
        cam_.trackbodyid = cfg_camera.track_body_id;
    } else {
        cam_.type = mjCAMERA_FREE;
    }

    // 初始化可视化选项
    mjv_defaultOption(&opt_);
    opt_.flags[mjVIS_CONTACTPOINT] = cfg_visualization.show_contact_point ? 1 : 0;
    opt_.flags[mjVIS_CONTACTFORCE] = cfg_visualization.show_contact_force ? 1 : 0;
    opt_.flags[mjVIS_COM] = cfg_visualization.show_com ? 1 : 0;
    opt_.flags[mjVIS_PERTFORCE] = 0;
    opt_.flags[mjVIS_TRANSPARENT] = 0;

    // 创建场景和渲染上下文
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);
    mjv_makeScene(mj_model_, &scn_, 2000);
    mjr_makeContext(mj_model_, &con_, mjFONTSCALE_150);

    // 设置回调函数
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, device_keyboardCallback);
    glfwSetCursorPosCallback(window_, device_mouseMoveCallback);
    glfwSetMouseButtonCallback(window_, device_mouseButtonCallback);
    glfwSetScrollCallback(window_, scrollCallback);

    // ImGui 初始化（在 OpenGL 上下文创建之后）
    if (cfg_imgui.enable_imgui) {
        initImGui();
        loadImGuiConfig(cfg_imgui.config_path);
    }

    // 轨迹可视化初始化
    trajectory_buffer_.reserve(cfg_visualization.trajectory_max_points);
    baseBodyId_ = mj_name2id(mj_model_, mjOBJ_BODY, "base_link");

    is_initialized_ = true;
    return true;
}

// 更新场景
void UIInterface::update() {
    if (!is_initialized_ || !window_) {
        return;
    }

    // 清除单次按键状态
    clearKeyboardState();

    // 重置单步模式
    if (!cfg_simulation.is_continuous) {
        cfg_simulation.is_running = false;
    }

    // 获取帧缓冲区尺寸
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // 更新可视化选项（用户可能在运行时修改）
    opt_.flags[mjVIS_CONTACTPOINT] = cfg_visualization.show_contact_point ? 1 : 0;
    opt_.flags[mjVIS_CONTACTFORCE] = cfg_visualization.show_contact_force ? 1 : 0;
    opt_.flags[mjVIS_COM] = cfg_visualization.show_com ? 1 : 0;

    // 更新场景
    mjv_updateScene(mj_model_, mj_data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

    // 更新和渲染轨迹
    updateTrajectory();
    renderTrajectory();

    // 渲染
    mjr_render(viewport, &scn_, &con_);

    // 渲染HUD
    renderHUD();

    // ImGui 调试面板渲染
    if (cfg_imgui.enable_imgui && imgui_initialized_) {
        renderImGuiPanel();
    }

    // 交换缓冲区
    glfwSwapBuffers(window_);

    // 处理事件
    glfwPollEvents();
}

// 渲染HUD
void UIInterface::renderHUD() {
    if (!mj_data_) {
        return;
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // 获取位置
    double pos_x = mj_data_->qpos[0];
    double pos_y = mj_data_->qpos[1];
    double pos_z = mj_data_->qpos[2];

    // 获取四元数并转换为欧拉角
    double qw = mj_data_->qpos[3];
    double qx = mj_data_->qpos[4];
    double qy = mj_data_->qpos[5];
    double qz = mj_data_->qpos[6];

    double roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    double pitch = asin(2 * (qw * qy - qz * qx));
    double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    // 获取速度
    double vel_x = mj_data_->qvel[0];
    double vel_y = mj_data_->qvel[1];
    double vel_z = mj_data_->qvel[2];

    // 格式化文本
    char text_left[1000];
    char text_right[1000];

    std::sprintf(text_left,
                "Sim Time\n"
                "Position (X, Y, Z)\n"
                "Euler RPY (deg)\n"
                "Velocity (X, Y, Z)\n"
                "Status");

    std::sprintf(text_right,
                "%.3f s\n"
                "%.2f, %.2f, %.3f m\n"
                "%.1f, %.1f, %.1f\n"
                "%.2f, %.2f, %.2f m/s\n"
                "%s",
                mj_data_->time,
                pos_x, pos_y, pos_z,
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
                vel_x, vel_y, vel_z,
                cfg_simulation.is_running ? "Running" : "Paused");

    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, text_left, text_right, &con_);
}

// 关闭窗口
void UIInterface::close() {
    // 先清理 ImGui（需要在 OpenGL 上下文仍有效时释放）
    if (imgui_initialized_) {
        shutdownImGui();
    }

    if (is_initialized_) {
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
    }

    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }

    if (is_initialized_) {
        glfwTerminate();
        is_initialized_ = false;
    }
}

// 检查窗口是否应该关闭
bool UIInterface::shouldClose() const {
    return window_ ? glfwWindowShouldClose(window_) : true;
}

// 重置仿真
void UIInterface::resetSimulation() {
    if (mj_model_ && mj_data_) {
        mj_resetData(mj_model_, mj_data_);
        mj_forward(mj_model_, mj_data_);

        // 清除轨迹历史
        trajectory_buffer_.clear();
        trajectory_update_counter_ = 0;
    }
}

// 清除键盘状态（单次按键）
void UIInterface::clearKeyboardState() {
    // 只清除单次触发按键（Space/Backspace），连续按键（WASD/QE/H/J）由 PRESS/RELEASE 管理
    device_keyboard.key_space = false;
    device_keyboard.key_backspace = false;
}

// ========== 静态回调函数 ==========

void UIInterface::device_keyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
    UIInterface* ui = static_cast<UIInterface*>(glfwGetWindowUserPointer(window));
    if (ui) {
        ui->handleKeyboard(key, scancode, act, mods);
    }
}

void UIInterface::device_mouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
    UIInterface* ui = static_cast<UIInterface*>(glfwGetWindowUserPointer(window));
    if (ui) {
        ui->handleMouseButton(button, act, mods);
    }
}

void UIInterface::device_mouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    UIInterface* ui = static_cast<UIInterface*>(glfwGetWindowUserPointer(window));
    if (ui) {
        ui->handleMouseMove(xpos, ypos);
    }
}

void UIInterface::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    UIInterface* ui = static_cast<UIInterface*>(glfwGetWindowUserPointer(window));
    if (ui) {
        ui->handleScroll(xoffset, yoffset);
    }
}

// ========== 内部处理函数 ==========

void UIInterface::handleKeyboard(int key, int scancode, int act, int mods) {
    // ImGui 正在捕获键盘输入 → 不处理仿真快捷键, 避免打字干扰相机/控制
    if (imgui_initialized_ && ImGui::GetIO().WantCaptureKeyboard) {
        return;
    }

    // Backspace: 重置仿真
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        resetSimulation();
        device_keyboard.key_backspace = true;
    }

    // Space: 切换运行/暂停
    if (act == GLFW_RELEASE && key == GLFW_KEY_SPACE) {
        cfg_simulation.is_running = !cfg_simulation.is_running;
        cfg_simulation.is_continuous = true;
        device_keyboard.key_space = true;
    }

    // Period(.): 单步模式
    if (act == GLFW_RELEASE && key == GLFW_KEY_PERIOD) {
        cfg_simulation.is_running = true;
        cfg_simulation.is_continuous = false;
    }

    // W/A/S/D/Q/E/H/J 键 — 连续按键（按住时持续为 true）
    if (key == GLFW_KEY_W) {
        device_keyboard.key_w = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
    if (key == GLFW_KEY_A) {
        device_keyboard.key_a = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
    if (key == GLFW_KEY_S) {
        device_keyboard.key_s = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
    if (key == GLFW_KEY_D) {
        device_keyboard.key_d = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
    if (key == GLFW_KEY_Q) {
        device_keyboard.key_q = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
    if (key == GLFW_KEY_E) {
        device_keyboard.key_e = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
    if (key == GLFW_KEY_H) {
        device_keyboard.key_h = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
    if (key == GLFW_KEY_J) {
        device_keyboard.key_j = (act == GLFW_PRESS || act == GLFW_REPEAT);
    }
}

void UIInterface::handleMouseButton(int button, int act, int mods) {
    // ImGui 正在捕获鼠标 → 不处理相机控制
    if (imgui_initialized_ && ImGui::GetIO().WantCaptureMouse) {
        device_mouse.button_left   = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
        device_mouse.button_middle = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        device_mouse.button_right  = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
        return;
    }

    // 更新鼠标按键状态
    device_mouse.button_left = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    device_mouse.button_middle = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    device_mouse.button_right = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // 记录当前鼠标位置
    glfwGetCursorPos(window_, &last_device_mouse_x_, &last_device_mouse_y_);
    device_mouse.pos_x = last_device_mouse_x_;
    device_mouse.pos_y = last_device_mouse_y_;
}

void UIInterface::handleMouseMove(double xpos, double ypos) {
    // 如果 ImGui 正在捕获鼠标，不处理相机控制
    if (imgui_initialized_ && ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    // 无按键按下时不处理
    if (!device_mouse.button_left && !device_mouse.button_middle && !device_mouse.button_right) {
        return;
    }

    // 计算鼠标位移
    double dx = xpos - last_device_mouse_x_;
    double dy = ypos - last_device_mouse_y_;
    last_device_mouse_x_ = xpos;
    last_device_mouse_y_ = ypos;

    device_mouse.pos_x = xpos;
    device_mouse.pos_y = ypos;

    // 获取窗口尺寸
    int width, height;
    glfwGetWindowSize(window_, &width, &height);

    // 检查Shift键
    bool mod_shift = (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // 确定操作类型
    mjtMouse action;
    if (device_mouse.button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (device_mouse.button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // 移动相机
    mjv_moveCamera(mj_model_, action, dx / height, dy / height, &scn_, &cam_);
}

void UIInterface::handleScroll(double xoffset, double yoffset) {
    // 如果 ImGui 正在捕获鼠标，不处理相机缩放
    if (imgui_initialized_ && ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    // 模拟鼠标缩放
    mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn_, &cam_);
}

// ========== ImGui 相关实现 ==========

void UIInterface::initImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // 启用 docking（可选，方便窗口布局）
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // 设置样式
    ImGui::StyleColorsDark();

    // 初始化 GLFW + OpenGL3 后端
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    imgui_initialized_ = true;
    printf("[ImGui] 初始化完成\n");
}

void UIInterface::shutdownImGui() {
    if (!imgui_initialized_) return;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    imgui_initialized_ = false;
    printf("[ImGui] 已清理\n");
}

void UIInterface::renderImGuiPanel() {
    // 开始新帧
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // 创建调试面板窗口
    ImGui::SetNextWindowSize(ImVec2(imgui_window_width_, imgui_window_height_), ImGuiCond_FirstUseEver);
    ImGui::Begin(imgui_window_title_.c_str(), nullptr);

    if (!tunable_registry_ || tunable_registry_->size() == 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "未设置参数注册表或注册表为空");
        ImGui::Text("请在 main.cpp 中调用 ui.setTunableParamRegistry(&registry)");
        ImGui::End();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        return;
    }

    // 按 YAML 中定义的 group 顺序分组展示
    for (const auto& group_cfg : imgui_groups_config_) {
        ImGui::SetNextItemOpen(!group_cfg.collapsed, ImGuiCond_Once);
        if (ImGui::CollapsingHeader(group_cfg.name.c_str())) {
            for (const auto& param_cfg : group_cfg.params) {
                auto* param = tunable_registry_->findParam(param_cfg.name);
                if (!param) continue;  // YAML 中定义但未注册的参数，跳过

                ImGui::PushID(param->name.c_str());
                switch (param->type) {
                    case TunableParamRegistry::ParamType::FLOAT: {
                        float step = static_cast<float>((param->max_val - param->min_val) * 0.01);
                        if (step <= 0.0f) step = 0.01f;
                        ImGui::SetNextItemWidth(150.0f);
                        ImGui::InputFloat(param->label.c_str(),
                                          static_cast<float*>(param->data_ptr),
                                          step, step * 10.0f, param->format.c_str());
                        break;
                    }
                    case TunableParamRegistry::ParamType::DOUBLE: {
                        double step = (param->max_val - param->min_val) * 0.01;
                        if (step <= 0.0) step = 0.01;
                        ImGui::SetNextItemWidth(150.0f);
                        ImGui::InputDouble(param->label.c_str(),
                                           static_cast<double*>(param->data_ptr),
                                           step, step * 10.0, param->format.c_str());
                        break;
                    }
                    case TunableParamRegistry::ParamType::INT:
                        ImGui::SetNextItemWidth(150.0f);
                        ImGui::InputInt(param->label.c_str(),
                                        static_cast<int*>(param->data_ptr));
                        break;
                    case TunableParamRegistry::ParamType::BOOL:
                        ImGui::Checkbox(param->label.c_str(),
                                        static_cast<bool*>(param->data_ptr));
                        break;
                }
                ImGui::PopID();
            }
        }
    }

    // 兜底：显示注册了但未在 YAML 中配置的参数
    bool has_unconfigured = false;
    for (auto& param : tunable_registry_->getParams()) {
        bool found_in_yaml = false;
        for (const auto& gc : imgui_groups_config_) {
            for (const auto& pc : gc.params) {
                if (pc.name == param.name) { found_in_yaml = true; break; }
            }
            if (found_in_yaml) break;
        }
        if (!found_in_yaml) {
            has_unconfigured = true;
            break;
        }
    }
    if (has_unconfigured) {
        if (ImGui::CollapsingHeader("其他未配置参数")) {
            for (auto& param : tunable_registry_->getParams()) {
                bool found_in_yaml = false;
                for (const auto& gc : imgui_groups_config_) {
                    for (const auto& pc : gc.params) {
                        if (pc.name == param.name) { found_in_yaml = true; break; }
                    }
                    if (found_in_yaml) break;
                }
                if (found_in_yaml) continue;

                ImGui::PushID(param.name.c_str());
                switch (param.type) {
                    case TunableParamRegistry::ParamType::FLOAT:
                        ImGui::SetNextItemWidth(150.0f);
                        ImGui::InputFloat(param.label.c_str(),
                                          static_cast<float*>(param.data_ptr),
                                          0.01f, 0.1f, param.format.c_str());
                        break;
                    case TunableParamRegistry::ParamType::DOUBLE:
                        ImGui::SetNextItemWidth(150.0f);
                        ImGui::InputDouble(param.label.c_str(),
                                           static_cast<double*>(param.data_ptr),
                                           0.01, 0.1, param.format.c_str());
                        break;
                    case TunableParamRegistry::ParamType::INT:
                        ImGui::SetNextItemWidth(150.0f);
                        ImGui::InputInt(param.label.c_str(),
                                        static_cast<int*>(param.data_ptr));
                        break;
                    case TunableParamRegistry::ParamType::BOOL:
                        ImGui::Checkbox(param.label.c_str(),
                                        static_cast<bool*>(param.data_ptr));
                        break;
                }
                ImGui::PopID();
            }
        }
    }

    ImGui::End();

    // 渲染 ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool UIInterface::loadImGuiConfig(const std::string& config_path) {
    // 候选搜索路径（覆盖常见的运行目录）
    static const std::vector<std::string> kCandidatePaths = {
        "cfg/imgui.yaml",                                          // 当前目录
        "../cfg/imgui.yaml",                                       // 从 mujoco_interface/build
        "../third_party/mujoco_interface/cfg/imgui.yaml",          // 从 omni_sim/
        "third_party/mujoco_interface/cfg/imgui.yaml",             // 从项目根目录
        "../../third_party/mujoco_interface/cfg/imgui.yaml",       // 从 omni_sim/build
    };

    // 确定配置文件路径
    std::string path = config_path;
    if (path.empty()) {
        for (const auto& candidate : kCandidatePaths) {
            FILE* f = fopen(candidate.c_str(), "r");
            if (f) {
                fclose(f);
                path = candidate;
                break;
            }
        }
    }

    if (path.empty()) {
        fprintf(stderr, "[ImGui] 未找到配置文件 cfg/imgui.yaml，已尝试:\n");
        for (const auto& c : kCandidatePaths) {
            fprintf(stderr, "  - %s\n", c.c_str());
        }
        fprintf(stderr, "[ImGui] 请确保文件存在，或通过 ui.cfg_imgui.config_path 指定路径\n");
        return false;
    }

    try {
        YAML::Node config = YAML::LoadFile(path);

        // 解析 window 配置
        if (config["window"]) {
            auto w = config["window"];
            imgui_window_title_  = w["title"].as<std::string>("参数调试面板");
            imgui_window_width_  = w["width"].as<int>(420);
            imgui_window_height_ = w["height"].as<int>(600);
        }

        // 解析 groups 配置
        imgui_groups_config_.clear();
        if (config["groups"]) {
            for (const auto& group_node : config["groups"]) {
                ImGuiGroupConfig gc;
                gc.name      = group_node["name"].as<std::string>();
                gc.collapsed = group_node["collapsed"].as<bool>(false);

                if (group_node["params"]) {
                    for (const auto& param_node : group_node["params"]) {
                        ImGuiParamConfig pc;
                        pc.name    = param_node["name"].as<std::string>();
                        pc.label   = param_node["label"].as<std::string>(pc.name);
                        pc.min_val = param_node["min"].as<double>(0.0);
                        pc.max_val = param_node["max"].as<double>(1.0);
                        pc.format  = param_node["format"].as<std::string>("%.3f");
                        gc.params.push_back(pc);
                    }
                }

                imgui_groups_config_.push_back(gc);
            }
        }

        // 将 YAML 中的元数据写入注册表中匹配的参数
        if (tunable_registry_) {
            for (const auto& gc : imgui_groups_config_) {
                for (const auto& pc : gc.params) {
                    auto* param = tunable_registry_->findParam(pc.name);
                    if (param) {
                        param->label   = pc.label;
                        param->group   = gc.name;
                        param->min_val = pc.min_val;
                        param->max_val = pc.max_val;
                        param->format  = pc.format;
                    }
                }
            }
        }

        printf("[ImGui] 配置文件加载成功: %s (共 %zu 个分组)\n",
               path.c_str(), imgui_groups_config_.size());
    } catch (const YAML::Exception& e) {
        fprintf(stderr, "[ImGui] 配置文件解析失败: %s\n", e.what());
        fprintf(stderr, "[ImGui] 将使用注册表中的默认显示配置\n");
        return false;
    }

    return true;
}

// ========== 轨迹可视化实现 ==========

void UIInterface::updateTrajectory() {
    if (!cfg_visualization.show_trajectory) {
        return;
    }

    // 每N帧采样一次，避免过密
    trajectory_update_counter_++;
    if (trajectory_update_counter_ % 5 != 0) {
        return;
    }

    // 获取base_link的世界坐标位置
    if (baseBodyId_ < 0 || !mj_data_) {
        return;
    }

    std::array<double, 3> pos = {
        mj_data_->xpos[3 * baseBodyId_ + 0],
        mj_data_->xpos[3 * baseBodyId_ + 1],
        mj_data_->xpos[3 * baseBodyId_ + 2]
    };

    // 添加到缓冲区
    trajectory_buffer_.push_back(pos);

    // 限制缓冲区大小，FIFO队列
    if (trajectory_buffer_.size() > static_cast<size_t>(cfg_visualization.trajectory_max_points)) {
        trajectory_buffer_.erase(trajectory_buffer_.begin());
    }
}

void UIInterface::renderTrajectory() {
    if (!cfg_visualization.show_trajectory || trajectory_buffer_.size() < 2) {
        return;
    }

    // 检查场景容量
    int segments_needed = static_cast<int>(trajectory_buffer_.size()) - 1;
    if (scn_.ngeom + segments_needed >= scn_.maxgeom) {
        segments_needed = scn_.maxgeom - scn_.ngeom - 1;
        if (segments_needed <= 0) return;
    }

    // 轨迹颜色 (红色，半透明)
    float rgba[4] = {1.0f, 0.0f, 0.0f, 0.6f};
    float line_width = 2.0f;

    // 为每对连续点创建线段
    size_t start_idx = trajectory_buffer_.size() - segments_needed - 1;
    for (size_t i = start_idx; i < trajectory_buffer_.size() - 1; i++) {
        mjvGeom* geom = &scn_.geoms[scn_.ngeom];

        // 使用 mjv_connector 初始化线段
        mjv_connector(geom, mjGEOM_LINE, line_width,
                     trajectory_buffer_[i].data(),
                     trajectory_buffer_[i + 1].data());

        // 设置颜色
        geom->rgba[0] = rgba[0];
        geom->rgba[1] = rgba[1];
        geom->rgba[2] = rgba[2];
        geom->rgba[3] = rgba[3];

        scn_.ngeom++;
    }
}
