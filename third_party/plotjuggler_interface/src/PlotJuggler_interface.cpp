#include "PlotJuggler_interface.h"
#include <sstream>
#include <iomanip>
#include <algorithm>

// ==================== 构造 / 析构 ====================

PlotJugglerInterface::PlotJugglerInterface()
    : cfg_udp_host("127.0.0.1")
    , cfg_udp_port(9870)
    , cfg_rate_limit_hz(0.0)
    , cfg_auto_timestamp(true)
    , cfg_packet_prefix("")
    , is_initialized(false)
    , packets_sent(0)
    , packets_dropped(0)
    , sock_fd_(-1)
    , addr_resolved_(false)
    , user_timestamp_(0.0)
    , rate_limit_interval_(0.0)
{
    memset(&dest_addr_, 0, sizeof(dest_addr_));
}

PlotJugglerInterface::~PlotJugglerInterface()
{
    close();
}

// ==================== 生命周期 ====================

bool PlotJugglerInterface::initialize()
{
    if (is_initialized) return true;

    // 计算频率限制间隔
    if (cfg_rate_limit_hz > 0.0) {
        rate_limit_interval_ = 1.0 / cfg_rate_limit_hz;
    } else {
        rate_limit_interval_ = 0.0;
    }

    // 解析目标地址 (不致命: PlotJuggler 可能没启动, UDP 本身就是无连接的)
    if (!resolveAddress()) {
        std::cerr << "[PlotJugglerInterface] Warning: 无法解析地址 "
                  << cfg_udp_host << ":" << cfg_udp_port
                  << ", UDP 消息将被静默丢弃" << std::endl;
        addr_resolved_ = false;
        is_initialized = true;
        return true;
    }
    addr_resolved_ = true;

    // 创建 UDP socket
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "[PlotJugglerInterface] Warning: socket() 失败: "
                  << strerror(errno) << ", UDP 已禁用" << std::endl;
        sock_fd_ = -1;
        is_initialized = true;
        return true;
    }

    // 设为非阻塞, 防止 sendto() 阻塞仿真主循环
    int flags = fcntl(sock_fd_, F_GETFL, 0);
    fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

    last_flush_time_ = std::chrono::steady_clock::now();
    is_initialized = true;

    std::cout << "[PlotJugglerInterface] UDP 数据流已就绪 -> "
              << cfg_udp_host << ":" << cfg_udp_port << std::endl;
    return true;
}

void PlotJugglerInterface::update()
{
    if (!is_initialized || sock_fd_ < 0 || !addr_resolved_) {
        return;  // 条件不满足, 静默跳过
    }

    // 频率限制检查
    if (rate_limit_interval_ > 0.0) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_flush_time_;
        if (elapsed.count() < rate_limit_interval_) {
            packets_dropped++;
            return;
        }
    }

    flush();
}

void PlotJugglerInterface::close()
{
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
    addr_resolved_ = false;
    if (is_initialized) {
        std::cout << "[PlotJugglerInterface] 已关闭. 共发送 " << packets_sent
                  << " 个包, 跳过 " << packets_dropped << " 个包." << std::endl;
    }
    is_initialized = false;
}

// ==================== 变量管理 ====================

void PlotJugglerInterface::addVariable(const std::string& name)
{
    variables_[name];  // 默认插入 0.0
}

void PlotJugglerInterface::removeVariable(const std::string& name)
{
    variables_.erase(name);
}

void PlotJugglerInterface::clearVariables()
{
    variables_.clear();
}

bool PlotJugglerInterface::hasVariable(const std::string& name) const
{
    return variables_.find(name) != variables_.end();
}

int PlotJugglerInterface::variableCount() const
{
    return static_cast<int>(variables_.size());
}

// ==================== 赋值 ====================

void PlotJugglerInterface::setValue(const std::string& name, double value)
{
    auto it = variables_.find(name);
    if (it != variables_.end()) {
        it->second = value;
    }
    // 变量未注册时静默忽略
}

void PlotJugglerInterface::setValue(const std::string& name, float value)
{
    setValue(name, static_cast<double>(value));
}

void PlotJugglerInterface::setValues(const std::vector<std::string>& names,
                                      const std::vector<double>& values)
{
    size_t n = std::min(names.size(), values.size());
    for (size_t i = 0; i < n; ++i) {
        setValue(names[i], values[i]);
    }
}

void PlotJugglerInterface::setTimestamp(double t_seconds)
{
    user_timestamp_ = t_seconds;
}

void PlotJugglerInterface::flush()
{
    if (sock_fd_ < 0 || !addr_resolved_ || variables_.empty()) {
        return;
    }

    std::string json = buildJsonMessage();
    sendPacket(json);

    last_flush_time_ = std::chrono::steady_clock::now();
    packets_sent++;
}

// ==================== 内部辅助函数 ====================

bool PlotJugglerInterface::resolveAddress()
{
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;       // IPv4
    hints.ai_socktype = SOCK_DGRAM;    // UDP

    std::string port_str = std::to_string(cfg_udp_port);
    int ret = getaddrinfo(cfg_udp_host.c_str(), port_str.c_str(), &hints, &res);
    if (ret != 0) {
        return false;
    }

    memcpy(&dest_addr_, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    return true;
}

std::string PlotJugglerInterface::buildJsonMessage()
{
    // 格式: {"timestamp": 123.456, "var1": 1.0, "var2": 2.0}
    // PlotJuggler UDP Server 插件要求每行一个 JSON 对象

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    // 时间戳
    double ts = cfg_auto_timestamp ? getCurrentTimestamp() : user_timestamp_;
    oss << "{\"timestamp\":" << ts;

    // 变量键值对 (map 保证字母序)
    for (const auto& kv : variables_) {
        oss << ",\"" << cfg_packet_prefix << kv.first << "\":" << kv.second;
    }

    oss << "}";
    return oss.str();
}

void PlotJugglerInterface::sendPacket(const std::string& data)
{
    ssize_t sent = sendto(sock_fd_, data.c_str(), data.size(), 0,
                          (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
    // UDP 是 fire-and-forget, 静默忽略所有错误
    // (ECONNREFUSED 等常见错误在 PlotJuggler 未启动时是正常的)
    (void)sent;
}

double PlotJugglerInterface::getCurrentTimestamp() const
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}
