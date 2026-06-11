#ifndef TUNABLE_PARAM_REGISTRY_H
#define TUNABLE_PARAM_REGISTRY_H

#include <string>
#include <vector>

/**
 * @brief 可调参数注册表
 *
 * 设计理念:
 * - C++ 侧只负责注册变量指针（name → 内存地址），极简 API
 * - 所有显示配置（标签、分组、范围、格式）由 imgui.yaml 统一管理
 * - name 字段是连接 YAML 和 C++ 的桥梁，必须一致
 *
 * 使用示例:
 *   TunableParamRegistry registry;
 *   registry.registerDouble("vel_Kp", &controller.config.vel_pid.Kp);
 *   registry.registerDouble("vel_Ki", &controller.config.vel_pid.Ki);
 */
class TunableParamRegistry {
public:
    enum class ParamType {
        FLOAT,
        DOUBLE,
        INT,
        BOOL
    };

    struct Param {
        std::string name;       // 唯一标识符，与 imgui.yaml 中的 name 对应
        std::string label;      // ImGui 中显示的标签（来自 YAML，或 fallback 为 name）
        std::string group;      // 所属分组名（来自 YAML）
        void* data_ptr;         // 指向实际变量的指针
        ParamType type;         // 变量类型
        double min_val = 0.0;   // 最小值（来自 YAML）
        double max_val = 0.0;   // 最大值（来自 YAML）
        std::string format;     // 显示格式串，如 "%.3f"（来自 YAML）
    };

    // ========== 极简注册接口（只需要 name + 指针）==========

    /// @brief 注册 float 变量
    void registerFloat(const std::string& name, float* ptr);

    /// @brief 注册 double 变量
    void registerDouble(const std::string& name, double* ptr);

    /// @brief 注册 int 变量
    void registerInt(const std::string& name, int* ptr);

    /// @brief 注册 bool 变量
    void registerBool(const std::string& name, bool* ptr);

    // ========== 查询接口 ==========

    /// @brief 获取所有已注册的参数（可修改）
    std::vector<Param>& getParams();

    /// @brief 获取所有已注册的参数（只读）
    const std::vector<Param>& getParams() const;

    /// @brief 获取所有不重复的分组名（保持注册顺序）
    std::vector<std::string> getGroupNames() const;

    /// @brief 按名称查找参数，未找到返回 nullptr
    const Param* findParam(const std::string& name) const;

    /// @brief 按名称查找参数（可修改），未找到返回 nullptr
    Param* findParam(const std::string& name);

    /// @brief 获取已注册的参数数量
    size_t size() const;

    /// @brief 清空所有已注册的参数
    void clear();

private:
    std::vector<Param> params_;
};

#endif // TUNABLE_PARAM_REGISTRY_H
