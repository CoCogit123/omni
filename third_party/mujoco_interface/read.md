# mujoco_interface 使用说明

本库提供两个独立组件：

- **UIInterface** — 仿真窗口创建、渲染、键盘鼠标交互
- **MJ_Interface** — 通用的 MuJoCo 状态提取与扭矩控制层

## 一、UI_interface 使用方法

    #include "UI_interface.h"

### 初始化部分
    // 加载 MuJoCo 模型
#### ！！！此处要填入自己的xml_path
    std::string xml_path = argv[1];   
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, 1000);
    if (!model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return 1;
    }
    mjData* data = mj_makeData(model);
    std::cout << "✓ Model loaded (DOF: " << model->nv << ")" << std::endl;

    // 实例化 UIInterface
    UIInterface ui(model, data);

    // 配置窗口和相机（直接访问公共成员）
    ui.cfg_window.title = "Simplified API Test";
    ui.cfg_window.width = 1200;
    ui.cfg_window.height = 800;
    ui.cfg_camera.enable_tracking = true;
    ui.cfg_camera.track_body_id = 1;
    ui.cfg_camera.azimuth = 150.0;
    ui.cfg_camera.elevation = -16.0;
    ui.cfg_visualization.show_contact_point = true;
    ui.cfg_visualization.show_contact_force = true;

    // 初始化
    if (!ui.initialize()) {
        std::cerr << "Failed to initialize UI" << std::endl;
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    std::cout << "\n控制说明:" << std::endl;
    std::cout << "  Space - 暂停/继续" << std::endl;
    std::cout << "  Backspace - 重置" << std::endl;
    std::cout << "  W/A/S/D - 自定义控制" << std::endl;
    std::cout << "  鼠标 - 控制相机\n" << std::endl;
    // 时间同步
    double sim_time_multiplier = 1.0;
    auto start_real_time = std::chrono::high_resolution_clock::now();
    double start_sim_time = data->time;
### 循环部分
    while (!ui.shouldClose()) {
        // 时间同步
        auto current_real_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> real_elapsed = current_real_time - start_real_time;
        double target_sim_time = start_sim_time + real_elapsed.count() * sim_time_multiplier;

        // 如果暂停，重置时间基准
        if (!ui.cfg_simulation.is_running) {
            start_real_time = std::chrono::high_resolution_clock::now();
            start_sim_time = data->time;
        }

        // 物理步进
        while (data->time < target_sim_time && ui.cfg_simulation.is_running) {
            mj_step(model, data);
        }

        if (ui.device_keyboard.key_w) {
            std::cout << "W pressed!" << std::endl;
        }
        if (ui.device_keyboard.key_a) {
            std::cout << "A pressed!" << std::endl;
        }
        if (ui.device_keyboard.key_s) {
            std::cout << "S pressed!" << std::endl;
        }
        if (ui.device_keyboard.key_d) {
            std::cout << "D pressed!" << std::endl;
        }
        if (ui.device_keyboard.key_space) {
            std::cout << "Simulation " << (ui.cfg_simulation.is_running ? "resumed" : "paused") << std::endl;
        }
        if (ui.device_keyboard.key_backspace) {
            std::cout << "Simulation reset!" << std::endl;
        }

        // 更新场景
        ui.update();

        // 控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
### 关闭部分
    ui.close();
    mj_deleteData(data);
    mj_deleteModel(model);


## 二、MJ_interface 使用方法（通用版）

MJ_Interface 不再预定义任何关节/传感器名称，用户按需传入。

    #include "MJ_interface.h"

### 配置并初始化
    // 创建接口（仅保存指针）
    MJ_Interface mj_if(model, data);

#### ！！！此处要按你的模型设置关节名（必填，否则 motor_pos 等为空）
    mj_if.cfg_jointNames = {
        "leg1_1_joint", "leg1_2_joint", "leg1_3_joint",
        "leg2_1_joint", "leg2_2_joint", "leg2_3_joint",
        "leg3_1_joint", "leg3_2_joint", "leg3_3_joint",
        "leg4_1_joint", "leg4_2_joint", "leg4_3_joint"
    };

#### ！！！此处要按你的模型设置传感器名（可选，不设置则对应数据保持为 0）
    mj_if.cfg_baseName                = "base_link";
    mj_if.cfg_orientationSensorName   = "imu_quat";
    mj_if.cfg_gyroSensorName          = "imu_gyro";
    mj_if.cfg_accSensorName           = "imu_acc";
    mj_if.cfg_velSensorName           = "baselink-velocity";

    // 一次性查找所有 ID，未配置/未找到的输出警告
    mj_if.initialize();

### 每步更新
        // 更新机器人状态
        mj_if.updateSensorValues();

        // 下发扭矩
        std::vector<double> tau(mj_if.jointNum, 0.0);
        // ... 计算 tau ...
        mj_if.setMotorsTorque(tau);

### 公开数据成员说明

| 成员 | 来源 | 说明 |
|------|------|------|
| `motor_pos` | qpos | 关节位置，长度 = jointNum |
| `motor_vel` | qvel | 关节速度，长度 = jointNum |
| `motor_pos_Old` | - | 上一时刻关节位置 |
| `basePos[3]` | xpos | 基链接世界坐标（需 cfg_baseName） |
| `baseLinVel[3]` | 有限差分 | 基链接线速度（需 cfg_baseName） |
| `baseQuat[4]` | sensordata | 四元数 [x,y,z,w]（需 cfg_orientationSensorName） |
| `rpy[3]` | 四元数解算 | roll/pitch/yaw，yaw 解缠绕（需 cfg_orientationSensorName） |
| `baseAngVel[3]` | sensordata | 角速度（需 cfg_gyroSensorName） |
| `baseAcc[3]` | sensordata | 加速度（需 cfg_accSensorName） |


## 融合版本 同时使用

    #include "UI_interface.h"
    #include "MJ_interface.h"

### 初始化
####  ui和环境
    std::string xml_path = argv[1];   
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, 1000);
    if (!model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return 1;
    }
    mjData* data = mj_makeData(model);
    std::cout << "✓ Model loaded (DOF: " << model->nv << ")" << std::endl;

    // 实例化 UIInterface
    UIInterface ui(model, data);

    // 配置窗口和相机（直接访问公共成员）
    ui.cfg_window.title = "Simplified API Test";
    ui.cfg_window.width = 1200;
    ui.cfg_window.height = 800;
    ui.cfg_camera.enable_tracking = true;
    ui.cfg_camera.track_body_id = 1;
    ui.cfg_camera.azimuth = 150.0;
    ui.cfg_camera.elevation = -16.0;
    ui.cfg_visualization.show_contact_point = true;
    ui.cfg_visualization.show_contact_force = true;

    // 初始化
    if (!ui.initialize()) {
        std::cerr << "Failed to initialize UI" << std::endl;
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
#### 交互接口
    // 创建接口（仅保存指针）
    MJ_Interface mj_if(model, data);

##### ！！！此处要按你的模型设置关节名（必填，否则 motor_pos 等为空）
    mj_if.cfg_jointNames = {
        "leg1_1_joint", "leg1_2_joint", "leg1_3_joint",
        "leg2_1_joint", "leg2_2_joint", "leg2_3_joint",
        "leg3_1_joint", "leg3_2_joint", "leg3_3_joint",
        "leg4_1_joint", "leg4_2_joint", "leg4_3_joint"
    };

##### ！！！此处要按你的模型设置传感器名（可选，不设置则对应数据保持为 0）
    mj_if.cfg_baseName                = "base_link";
    mj_if.cfg_orientationSensorName   = "imu_quat";
    mj_if.cfg_gyroSensorName          = "imu_gyro";
    mj_if.cfg_accSensorName           = "imu_acc";
    mj_if.cfg_velSensorName           = "baselink-velocity";

    // 一次性查找所有 ID，未配置/未找到的输出警告
    mj_if.initialize();
#### 其他
    std::cout << "\n控制说明:" << std::endl;
    std::cout << "  Space - 暂停/继续" << std::endl;
    std::cout << "  Backspace - 重置" << std::endl;
    std::cout << "  W/A/S/D - 自定义控制" << std::endl;
    std::cout << "  鼠标 - 控制相机\n" << std::endl;
    // 时间同步
    double sim_time_multiplier = 1.0;
    auto start_real_time = std::chrono::high_resolution_clock::now();
    double start_sim_time = data->time;

### 循环

    while (!ui.shouldClose()) {
        // 时间同步
        auto current_real_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> real_elapsed = current_real_time - start_real_time;
        double target_sim_time = start_sim_time + real_elapsed.count() * sim_time_multiplier;

        // 如果暂停，重置时间基准
        if (!ui.cfg_simulation.is_running) {
            start_real_time = std::chrono::high_resolution_clock::now();
            start_sim_time = data->time;
        }

        // 物理步进
        while (data->time < target_sim_time && ui.cfg_simulation.is_running) {
            mj_step(model, data);

            // 更新机器人状态
            mj_if.updateSensorValues();

            // 下发扭矩
            std::vector<double> tau(mj_if.jointNum, 0.0);
            mj_if.setMotorsTorque(tau);

        }

        // 更新场景
        ui.update();

        // 控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

### 关闭部分
    ui.close();
    mj_deleteData(data);
    mj_deleteModel(model);

## 编译链接库

    target_link_libraries链接编译后的libui_interface 和 libmj_interface .a
    link_directories该文件夹下的include文件夹

## 其他工具

### 施加外部力矩

// =================================================================
// 自定义控制回调函数（每次 mj_step 底层计算动力学前都会自动触发）
// =================================================================
void outer_push(const mjModel* m, mjData* d,device_keyboard key) {
    // 1. 获取你要施加推力的车身 Body ID
    // mjOBJ_BODY 表示你要找的是 body 节点
    int chassis_id = mj_name2id(m, mjOBJ_BODY, "base_link");
    
    if (chassis_id == -1) {
        std::cerr << "错误：找不到名为 base_link 的刚体！" << std::endl;
        return;
    }

    // 2. 清空上一帧的所有外力！
    // mju_zero 是 MuJoCo 自带的极速内存清零宏，比 memset 更好用
    mju_zero(d->xfrc_applied, 6 * m->nbody);

    // 3. 构造扰动力
    // 假设：我们要模拟一阵沿 Y 轴的 20N 强侧风推力，外加一个绕 Z 轴的 1.5Nm 扭矩
    
    // Fx (X轴推力)
    d->xfrc_applied[6 * chassis_id + 0] = 0.0;
    // Fy (Y轴推力)
    d->xfrc_applied[6 * chassis_id + 1] = 20.0;
    // Fz (Z轴推力)
    d->xfrc_applied[6 * chassis_id + 2] = 0.0;
    // Tx (X轴扭矩)
    d->xfrc_applied[6 * chassis_id + 3] = 0.0;
    // Ty (Y轴扭矩)
    d->xfrc_applied[6 * chassis_id + 4] = 0.0;
    // Tz (Z轴扭矩)
    d->xfrc_applied[6 * chassis_id + 5] = 1.5;

    // 当然，如果你有算好的控制律输出 (比如 LQR 算出的电机扭矩)
    // 也是在这个函数里写给 d->ctrl
    // int motor_id = mj_name2id(m, mjOBJ_ACTUATOR, "motor_wheel1");
    // d->ctrl[motor_id] = your_calculated_torque;
}


## 三、ImGui 实时调参面板

UI_interface 内置了 Dear ImGui 支持，可以在仿真运行时通过滑块、复选框实时调节参数，无需重新编译。

### 3.1 快速开启/关闭

```cpp
// 开启 ImGui 调试面板
ui.cfg_imgui.enable_imgui = true;

// 关闭 ImGui 调试面板
ui.cfg_imgui.enable_imgui = false;
```

**注意**: `enable_imgui` 必须在 `ui.initialize()` 调用**之前**设置，因为 ImGui 的初始化在 `initialize()` 中完成。

### 3.2 两步配置流程

#### 第一步：在 `cfg/imgui.yaml` 中完成所有配置

配置文件位于 `mujoco_interface/cfg/imgui.yaml`，是调参的**唯一配置来源**。所有显示标签、取值范围、格式、分组都在这里定义：

```yaml
window:
  title: "参数调试面板"   # 面板窗口标题
  width: 420              # 窗口宽度
  height: 600             # 窗口高度
  init_collapsed: false   # 启动时是否折叠

groups:
  - name: "速度环 PID"    # 分组名称
    collapsed: false       # 启动时是否折叠此分组
    params:
      - name: "vel_Kp"     # 参数唯一标识（与 C++ 注册名对应）
        label: "Kp"        # 在面板中显示的标签
        min: 0.0           # 滑块最小值
        max: 100.0          # 滑块最大值
        format: "%.2f"     # 数值显示格式
      - name: "vel_Ki"
        label: "Ki"
        min: 0.0
        max: 50.0
        format: "%.2f"
      - name: "vel_Kd"
        label: "Kd"
        min: 0.0
        max: 20.0
        format: "%.2f"
  # ... 更多分组 ...
```

**核心原则**: YAML 是**唯一的配置来源**，C++ 只负责把变量指针按 name 关联上。

#### 第二步：在 C++ 中注册变量指针（极简，只需 name + 指针）

```cpp
#include "tunable_param_registry.h"

// 创建注册表
TunableParamRegistry tunable_params;

// 注册需要调参的变量（name 必须与 YAML 中的 name 一致）
tunable_params.registerDouble("vel_Kp", &controller.config.vel_pid.Kp);
tunable_params.registerDouble("vel_Ki", &controller.config.vel_pid.Ki);
tunable_params.registerDouble("vel_Kd", &controller.config.vel_pid.Kd);

// 支持的注册方法（只有 name + 指针两个参数）:
//   registerFloat(name, ptr)
//   registerDouble(name, ptr)
//   registerInt(name, ptr)
//   registerBool(name, ptr)

// 传递给 UIInterface 并开启（必须在 ui.initialize() 之前）
ui.setTunableParamRegistry(&tunable_params);
ui.cfg_imgui.enable_imgui = true;
ui.initialize();
```

> 如果要新增/删除调参变量、调整范围或修改标签，**只需编辑 imgui.yaml，无需改动 C++ 代码**。

### 3.3 完整示例

```cpp
#include "UI_interface.h"
#include "MJ_interface.h"
#include "tunable_param_registry.h"

int main() {
    // ... MuJoCo 模型加载、控制器初始化 ...

    UIInterface ui(model, data);
    ui.cfg_window.title = "My Simulation";

    // ---- ImGui 调参配置（必须在 initialize() 之前）----
    TunableParamRegistry tunable_params;

    // 只需注册 name + 指针，标签/范围/格式全在 imgui.yaml 中配置
    tunable_params.registerDouble("vel_Kp", &controller.config.vel_pid.Kp);
    tunable_params.registerDouble("vel_Ki", &controller.config.vel_pid.Ki);
    tunable_params.registerDouble("vel_Kd", &controller.config.vel_pid.Kd);

    double sim_speed = 1.0;
    tunable_params.registerDouble("sim_time_multiplier", &sim_speed);

    // 传递给 UI 并开启
    ui.setTunableParamRegistry(&tunable_params);
    ui.cfg_imgui.enable_imgui = true;

    ui.initialize();
    // ---- ImGui 配置结束 ----

    // 主循环（ImGui 在 ui.update() 中自动渲染）
    while (!ui.shouldClose()) {
        // ... 物理步进 ...
        ui.update();
    }

    ui.close();
    return 0;
}
```

### 3.4 工作流程总结

```
  imgui.yaml (唯一配置来源)          main.cpp (只做关联)
  ┌──────────────────────┐          ┌──────────────────────────┐
  │ groups:              │          │ TunableParamRegistry r;  │
  │  - name: "速度环"    │          │ r.registerDouble(        │
  │    params:           │  name    │   "vel_Kp", &kp);        │
  │    - name: vel_Kp    │◄─匹配──►│ r.registerDouble(        │
  │      label: "Kp"     │          │   "vel_Ki", &ki);        │
  │      min: 0.0        │          └───────────┬──────────────┘
  │      max: 100.0      │                      │
  │      format: "%.2f"  │          ┌───────────▼──────────────┐
  └──────────┬───────────┘          │ YAML 元数据写入注册表     │
             │                      │ → param.label = "Kp"     │
             ▼                      │ → param.group = "速度环" │
  UIInterface::loadImGuiConfig()    └───────────┬──────────────┘
             │                      ┌───────────▼──────────────┐
             └──────────────────────►│ renderImGuiPanel()      │
                                    │ → InputDouble("Kp",...)  │
                                    │ → InputDouble("Ki",...)  │
                                    └──────────────────────────┘
```

### 3.5 注意事项

| 项目 | 说明 |
|------|------|
| **单一配置源** | YAML 是唯一的显示配置来源。新增/删除参数、调整范围、修改标签只需编辑 `cfg/imgui.yaml`，无需改动 C++ |
| **生命周期** | `TunableParamRegistry` 和注册变量的生命周期必须长于 `UIInterface`（放在 `main()` 中即可） |
| **注册顺序** | 变量注册和 `cfg_imgui.enable_imgui` 必须在 `ui.initialize()` **之前**完成 |
| **配置文件路径** | 默认自动多路径查找（`cfg/imgui.yaml` → `../cfg/` → `../third_party/mujoco_interface/cfg/`），也可通过 `cfg_imgui.config_path` 手动指定 |
| **类型支持** | `registerFloat` / `registerDouble` / `registerInt`（输入框）/ `registerBool`（复选框） |
| **编译依赖** | `sudo apt install libyaml-cpp-dev` |