# plotjuggler_interface 使用说明

本库提供 **PlotJuggler 实时数据流接口**，通过 UDP 将仿真/程序数据以 JSON 格式发送到 PlotJuggler，
实现数据的实时可视化调试。无需 ROS，无需外部依赖，仅使用 C++ 标准库和系统 socket。

- **PlotJugglerInterface** — UDP 数据流发送，支持变量注册、动态赋值、频率限制、自定义时间戳

---

## 一、PlotJuggler 端配置 (一次性)

1. 安装并启动 PlotJuggler
2. 启用 UDP Server 插件:
   - 菜单 `App` → `Plugins` → 勾选 **UDP Server**
3. 默认监听端口 `9870` (无需修改)
4. 数据到达后，左侧面板会出现所有变量名，拖拽到右侧绘图区即可实时可视化

---

## 二、编译

```bash
cd third_party/plotjuggler_interface
mkdir -p build && cd build
cmake ..
make
```

编译产物: `build/libplotjuggler_interface.a` (静态库)

如需集成到自己的 CMake 项目:

```cmake
# include 路径
include_directories(${PLOTJUGGLER_INTERFACE_DIR}/include)

# 链接静态库
link_directories(${PLOTJUGGLER_INTERFACE_DIR}/build)
target_link_libraries(your_target
    ${PLOTJUGGLER_INTERFACE_DIR}/build/libplotjuggler_interface.a
)
```

---

## 三、PlotJugglerInterface 使用方法

```cpp
#include "PlotJuggler_interface.h"
```

### 3.1 创建与配置

```cpp
PlotJugglerInterface pj_if;

// ---- 配置 (在 initialize() 之前设置) ----
pj_if.cfg_udp_host = "127.0.0.1";    // PlotJuggler 所在 IP, 默认本机
pj_if.cfg_udp_port = 9870;           // UDP Server 插件端口, 默认 9870
pj_if.cfg_rate_limit_hz = 50.0;      // 每秒最多发送 50 次 (0 = 不限频)
pj_if.cfg_auto_timestamp = false;     // false = 使用仿真时间; true = 用系统时钟
pj_if.cfg_packet_prefix = "bot.";    // 可选: 变量名前缀, 如 "bot.base_pos_x"
```

### 3.2 注册变量

```cpp
// 注册需要发送到 PlotJuggler 的变量 (可以在 initialize 之前或之后)
pj_if.addVariable("base_pos_x");
pj_if.addVariable("base_pos_y");
pj_if.addVariable("base_pos_z");
pj_if.addVariable("base_roll");
pj_if.addVariable("base_pitch");
pj_if.addVariable("base_yaw");
pj_if.addVariable("base_linvel_x");
pj_if.addVariable("base_linvel_y");
pj_if.addVariable("base_angvel_z");
pj_if.addVariable("motor_pos_1");
pj_if.addVariable("motor_vel_1");
pj_if.addVariable("motor_tau_1");
// ... 按需添加 ...

// 也支持中途动态添加/移除
pj_if.removeVariable("不需要的变量");
pj_if.clearVariables();  // 清空全部
```

### 3.3 初始化

```cpp
pj_if.initialize();
// 注意: initialize() 即使 PlotJuggler 未启动也会返回 true
// UDP 是无连接的, 消息发不出去就静默丢弃, 不影响程序运行
```

### 3.4 主循环中使用

```cpp
while (running) {
    // ... 你的计算 / 仿真步进 ...

    // 方式一: 逐变量赋值 (推荐, 最直观)
    pj_if.setTimestamp(sim_time);              // 用仿真时间 (cfg_auto_timestamp=false 时)
    pj_if.setValue("base_pos_x",  pos[0]);
    pj_if.setValue("base_pos_y",  pos[1]);
    pj_if.setValue("base_pitch",   rpy[1]);
    pj_if.setValue("motor_tau_1",  tau[0]);

    // 方式二: 批量赋值
    std::vector<std::string> names = {"base_pos_x", "base_pos_y", "base_pos_z"};
    std::vector<double>      vals  = {pos[0], pos[1], pos[2]};
    pj_if.setValues(names, vals);

    // 每帧调用 update() — 内部按 cfg_rate_limit_hz 决定是否真的发送
    pj_if.update();

    // 如需立即发送 (忽略频率限制), 调用 flush()
    // pj_if.flush();
}
```

### 3.5 关闭

```cpp
pj_if.close();
// 打印统计: 共发送 X 个包, 跳过 Y 个包
```

### 3.6 完整最小示例

```cpp
#include "PlotJuggler_interface.h"
#include <thread>
#include <cmath>

int main() {
    PlotJugglerInterface pj_if;

    // 配置
    pj_if.cfg_auto_timestamp = true;    // 用系统时钟
    pj_if.cfg_rate_limit_hz = 20.0;    // 每秒 20 次

    // 注册变量
    pj_if.addVariable("sin_wave");
    pj_if.addVariable("cos_wave");
    pj_if.addVariable("counter");

    pj_if.initialize();

    double t = 0.0;
    for (int i = 0; i < 500; ++i) {
        pj_if.setValue("sin_wave", std::sin(t));
        pj_if.setValue("cos_wave", std::cos(t));
        pj_if.setValue("counter", static_cast<double>(i));

        pj_if.update();

        t += 0.01;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    pj_if.close();
    return 0;
}
```

---

## 四、UDP JSON 数据格式

每条 UDP 消息是一个单行 JSON:

```json
{"timestamp":1748961234.567890,"bot.base_pos_x":0.123456,"bot.base_pitch":0.034907}
```

PlotJuggler 的 UDP Server 插件解析每个 key 为一条独立的时间序列曲线，`timestamp` 为 X 轴时间。

---

## 五、公开接口速查表

### 配置成员 (initialize 前设置)

| 成员 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `cfg_udp_host` | string | `"127.0.0.1"` | PlotJuggler 所在 IP |
| `cfg_udp_port` | int | `9870` | UDP 端口 |
| `cfg_rate_limit_hz` | double | `0.0` | 最大发送频率 (Hz), 0=不限 |
| `cfg_auto_timestamp` | bool | `true` | true=系统时间, false=用 setTimestamp() |
| `cfg_packet_prefix` | string | `""` | 变量名前缀 |

### 状态成员 (只读)

| 成员 | 类型 | 说明 |
|------|------|------|
| `is_initialized` | bool | 是否已初始化 |
| `packets_sent` | int | 已发送包计数 |
| `packets_dropped` | int | 被频率限制跳过的包计数 |

### 方法

| 方法 | 说明 |
|------|------|
| `initialize()` | 创建 UDP socket，返回 true |
| `update()` | 每帧调用，按频率限制自动发送 |
| `close()` | 关闭 socket，打印统计 |
| `addVariable(name)` | 注册变量 |
| `removeVariable(name)` | 移除变量 |
| `clearVariables()` | 清空所有变量 |
| `hasVariable(name)` | 检查变量是否存在 |
| `variableCount()` | 返回当前变量数量 |
| `setValue(name, val)` | 设置变量值 (double/float) |
| `setValues(names, vals)` | 批量设置 |
| `setTimestamp(t)` | 手动设置时间戳 |
| `flush()` | 强制立即发送 |

---

## 六、注意事项

1. **线程安全**: 本类不含互斥锁，所有方法应从同一线程调用（通常是主线程）。如需多线程访问，请外部加锁。
2. **UDP 包大小**: 单条消息应在 ~1400 字节以内（含 JSON 开销约 80 字节，每个变量约 20 字节，即最多约 60-70 个变量）。如需更多变量，可创建多个 `PlotJugglerInterface` 实例用不同端口。
3. **PlotJuggler 未启动**: 不影响程序运行，UDP 消息静默丢弃，socket 设为非阻塞不会卡住。
4. **频率限制**: 推荐设置 `cfg_rate_limit_hz = 20~50`，避免 UDP 包过密导致 PlotJuggler 渲染卡顿。
5. **C++ 标准**: C++14，与 `mujoco_interface` 保持一致。
