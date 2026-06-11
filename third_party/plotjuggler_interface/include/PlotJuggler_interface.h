#ifndef PLOTJUGGLER_INTERFACE_H
#define PLOTJUGGLER_INTERFACE_H

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

/**
 * @brief PlotJuggler 实时数据流接口
 *
 * 通过 UDP 将仿真数据以 JSON 格式发送到 PlotJuggler 的 UDP Server 插件，
 * 实现仿真数据的实时可视化调试。
 *
 * 使用方式：
 *   1. 设置 cfg_ 开头的配置成员
 *   2. 调用 addVariable() 注册需要发送的变量
 *   3. 调用 initialize() 创建 UDP socket
 *   4. 每帧调用 setValue() 更新变量值，然后调用 update() 自动按频率限制发送
 *   5. 程序结束时调用 close()
 */
class PlotJugglerInterface
{
public:
    // ==================== 配置成员 (initialize 之前设置) ====================
    std::string cfg_udp_host;        // PlotJuggler UDP Server 所在 IP, 默认 127.0.0.1
    int         cfg_udp_port;        // UDP 端口, 默认 9870 (PlotJuggler UDP Server 插件默认端口)
    double      cfg_rate_limit_hz;   // 最大发送频率 (Hz), 0 = 不限制, 每帧都发
    bool        cfg_auto_timestamp;  // true = 自动使用系统时钟; false = 使用 setTimestamp() 手动设置
    std::string cfg_packet_prefix;   // 变量名前缀, 如 "ballbot." 可避免多实例变量名冲突

    // ==================== 状态成员 (只读) ====================
    bool is_initialized;   // UDP socket 是否已创建
    int  packets_sent;     // 已发送的 UDP 包计数
    int  packets_dropped;  // 因频率限制跳过的包计数

    // ==================== 构造 / 析构 ====================
    PlotJugglerInterface();
    ~PlotJugglerInterface();

    // ==================== 生命周期 ====================
    bool initialize();     // 创建 UDP socket, 即使 PlotJuggler 未启动也返回 true
    void update();         // 每帧调用: 根据频率限制决定是否发送数据
    void close();          // 关闭 socket, 打印统计信息

    // ==================== 变量管理 ====================
    void addVariable(const std::string& name);                   // 注册变量
    void removeVariable(const std::string& name);                // 移除变量
    void clearVariables();                                       // 清空所有变量
    bool hasVariable(const std::string& name) const;             // 检查变量是否存在
    int  variableCount() const;                                  // 当前变量数量

    // ==================== 赋值 ====================
    void setValue(const std::string& name, double value);        // 单变量赋值
    void setValue(const std::string& name, float value);         // 便捷重载 (float → double)
    void setValues(const std::vector<std::string>& names,        // 批量赋值
                   const std::vector<double>& values);
    void setTimestamp(double t_seconds);                         // 设置仿真时间 (cfg_auto_timestamp=false 时生效)
    void flush();                                                // 强制立即发送 (忽略频率限制)

private:
    // ==================== 网络相关 ====================
    int                sock_fd_;         // UDP socket 文件描述符, -1 表示未打开
    struct sockaddr_in dest_addr_;       // 预解析的目标地址
    bool               addr_resolved_;   // 地址是否解析成功

    // ==================== 数据存储 ====================
    std::map<std::string, double> variables_;  // 变量名 → 当前值 (map 保证按 key 字母序输出)
    double user_timestamp_;                     // 用户设置的仿真时间

    // ==================== 频率限制 ====================
    std::chrono::steady_clock::time_point last_flush_time_;
    double rate_limit_interval_;  // 预计算: 1.0 / cfg_rate_limit_hz

    // ==================== 内部辅助函数 ====================
    bool        resolveAddress();
    std::string buildJsonMessage();
    void        sendPacket(const std::string& data);
    double      getCurrentTimestamp() const;
};

#endif // PLOTJUGGLER_INTERFACE_H
