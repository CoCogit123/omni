#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <array>

// 前向声明
class TunableParamRegistry;

/**
 * @brief ImGui 参数显示配置（从 YAML 解析）
 */
struct ImGuiParamConfig {
    std::string name;
    std::string label;
    double min_val = 0.0;
    double max_val = 1.0;
    std::string format = "%.3f";
};

/**
 * @brief ImGui 分组显示配置（从 YAML 解析）
 */
struct ImGuiGroupConfig {
    std::string name;
    bool collapsed = false;
    std::vector<ImGuiParamConfig> params;
};

/**
 * @brief MuJoCo UI 可视化接口类
 *
 * 设计理念：
 * - 所有用户需要的数据都是公共成员变量，直接访问
 * - 函数只提供核心功能：构造、初始化、更新、关闭
 * - 内部实现细节完全私有
 */
class UIInterface {
public:
    // ========== 公共数据结构 ==========

    /**
     * @brief 键盘状态（直接访问：ui.device_keyboard.key_w）
     */
    struct {
        bool key_w{false};
        bool key_s{false};
        bool key_a{false};
        bool key_d{false};
        bool key_q{false};
        bool key_e{false};
        bool key_h{false};
        bool key_j{false};
        bool key_space{false};
        bool key_backspace{false};
    } device_keyboard;

    /**
     * @brief 鼠标状态（直接访问：ui.device_mouse.button_left）
     */
    struct {
        bool button_left{false};
        bool button_middle{false};
        bool button_right{false};
        double pos_x{0.0};
        double pos_y{0.0};
    } device_mouse;

    /**
     * @brief 仿真控制（直接访问：ui.cfg_simulation.is_running）
     */
    struct {
        bool is_running{true};      // 仿真是否运行
        bool is_continuous{false};   // 是否连续运行（false为单步模式）
    } cfg_simulation;

    /**
     * @brief 相机配置（直接访问：ui.cfg_camera.azimuth）
     */
    struct {
        double azimuth{150.0};          // 方位角
        double elevation{-16.0};        // 仰角
        double distance{3.0};           // 距离
        double lookat_x{0.0};           // 观察点X
        double lookat_y{0.0};           // 观察点Y
        double lookat_z{1.0};           // 观察点Z
        bool enable_tracking{true};     // 是否启用跟踪模式
        int track_body_id{1};           // 跟踪的刚体ID
    } cfg_camera;

    /**
     * @brief 可视化选项（直接访问：ui.cfg_visualization.show_contact）
     */
    struct {
        bool show_contact_point{false};  // 显示接触点
        bool show_contact_force{false};  // 显示接触力
        bool show_com{false};            // 显示质心
        bool show_trajectory{true};      // 显示轨迹
        int trajectory_max_points{500};  // 轨迹最大点数
    } cfg_visualization;

    /**
     * @brief 窗口配置（直接访问：ui.cfg_window.width）
     */
    struct {
        std::string title{"MuJoCo Simulation"};
        int width{1200};
        int height{800};
    } cfg_window;

    /**
     * @brief ImGui 调试面板配置（直接访问：ui.cfg_imgui.enable_imgui）
     *
     * 使用示例:
     *   ui.cfg_imgui.enable_imgui = true;
     *   ui.cfg_imgui.config_path = "cfg/imgui.yaml";
     */
    struct {
        bool enable_imgui{false};           // 是否启用 ImGui 调试面板
        std::string config_path{""};        // imgui.yaml 路径（空则自动查找 cfg/imgui.yaml）
    } cfg_imgui;

    // ========== 核心函数 ==========

    /**
     * @brief 构造函数
     * @param model MuJoCo模型指针
     * @param data MuJoCo数据指针
     */
    UIInterface(mjModel* model, mjData* data);

    /**
     * @brief 析构函数（自动清理资源）
     */
    ~UIInterface();

    /**
     * @brief 初始化并创建窗口
     * @return 成功返回true，失败返回false
     *
     * 使用方法：
     *   ui.camera.enable_tracking = true;
     *   ui.window.width = 1920;
     *   ui.initialize();
     */
    bool initialize();

    /**
     * @brief 更新场景（渲染一帧）
     *
     * 使用方法：
     *   while (!glfwWindowShouldClose(window)) {
     *       if (ui.simulation.is_running) {
     *           mj_step(model, data);
     *       }
     *       ui.update();
     *   }
     */
    void update();

    /**
     * @brief 关闭窗口并释放资源
     */
    void close();

    /**
     * @brief 检查窗口是否应该关闭
     * @return 应该关闭返回true
     */
    bool shouldClose() const;

    /**
     * @brief 获取 GLFW 窗口指针（用于 glfwGetKey 等直接轮询）
     */
    GLFWwindow* getWindow() const { return window_; }

    /**
     * @brief 重置仿真
     */
    void resetSimulation();

    /**
     * @brief 设置可调参数注册表（用于 ImGui 实时调参）
     * @param registry 参数注册表指针（生命周期需长于 UIInterface）
     *
     * 使用示例:
     *   TunableParamRegistry registry;
     *   registry.registerFloat("vel_Kp", &kp, 0.0f, 100.0f, "Kp", "速度环 PID");
     *   ui.setTunableParamRegistry(&registry);
     */
    void setTunableParamRegistry(TunableParamRegistry* registry);

private:
    // MuJoCo 数据
    mjModel* mj_model_;
    mjData* mj_data_;

    // GLFW 窗口
    GLFWwindow* window_;

    // MuJoCo 可视化对象
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;

    // 鼠标交互
    double last_device_mouse_x_;
    double last_device_mouse_y_;

    // 初始化标志
    bool is_initialized_;

    // ========== ImGui 相关私有成员 ==========

    TunableParamRegistry* tunable_registry_{nullptr};
    bool imgui_initialized_{false};

    // 从 YAML 解析的显示配置
    std::vector<ImGuiGroupConfig> imgui_groups_config_;
    std::string imgui_window_title_{"参数调试面板"};
    int imgui_window_width_{420};
    int imgui_window_height_{600};

    // ========== 静态回调函数（GLFW要求）==========
    static void device_keyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void device_mouseButtonCallback(GLFWwindow* window, int button, int act, int mods);
    static void device_mouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    // 内部辅助函数
    void handleKeyboard(int key, int scancode, int act, int mods);
    void handleMouseButton(int button, int act, int mods);
    void handleMouseMove(double xpos, double ypos);
    void handleScroll(double xoffset, double yoffset);
    void renderHUD();
    void clearKeyboardState();

    // 轨迹可视化方法
    void updateTrajectory();
    void renderTrajectory();

    // ImGui 相关方法
    void initImGui();
    void renderImGuiPanel();
    void shutdownImGui();
    bool loadImGuiConfig(const std::string& config_path);

    // 轨迹可视化数据
    std::vector<std::array<double, 3>> trajectory_buffer_;  // 存储 [x, y, z] 历史位置
    int trajectory_update_counter_{0};  // 用于控制采样频率
    int baseBodyId_{-1};  // base_link的body ID
};

#endif // UI_INTERFACE_H
