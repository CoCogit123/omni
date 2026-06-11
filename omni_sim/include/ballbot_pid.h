#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace ballbot {

// =====================================================================
//                     PID 参数 & 运动学配置
// =====================================================================

/// @brief 单轴 PID 参数
struct PIDParams {
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double integral_max = 0.0;   // 积分抗饱和上限 (0 = 不限)
    double output_max   = 0.0;   // 输出限幅上限   (0 = 不限)
};

/// @brief 单个全向轮的运动学配置 (用于雅可比计算)
struct WheelKinematicConfig {
    Eigen::Vector3d contact_position_imu  = Eigen::Vector3d::Zero();    // IMU原点→接触点Pi
    Eigen::Vector3d ball_to_contact_imu   = Eigen::Vector3d::Zero();    // 球心→接触点Pi
    Eigen::Matrix3d rotation_wheel_to_imu = Eigen::Matrix3d::Identity();// 轮系→IMU系 R_i
    Eigen::Vector3d active_direction_wheel = Eigen::Vector3d::UnitY();  // 驱动方向(Y轴)
};

struct BallbotKinematicConfig {
    double ball_radius          = 0.15;   // 球半径 r_b
    double wheel_radius         = 0.048;  // 轮有效半径 r_w
    double imu_to_ball_center_z = 0.183;  // IMU→球心Z向距离 h
    std::vector<WheelKinematicConfig> wheels;
};

// =====================================================================
//                     运动学雅可比
// =====================================================================

/// @brief 创建 omni_ballbot 默认运动学配置
BallbotKinematicConfig createOmniBallbotConfig();

/// @brief 计算雅可比矩阵: 轮速 = J * 机体速度
Eigen::Matrix3d computeBallbotJacobian(const BallbotKinematicConfig& config);

// =====================================================================
//               PID 运行时数据 (暴露给外部)
// =====================================================================
//
// 每个 PID 环实时暴露 target / actual / error / P / I / D / integral / output,
// 维度由模板参数 N 决定: PIDState<2> = 2D, PIDState<1> = 1D, PIDState<3> = 3D.
//
// 使用方式: 读取 controller.state.vel.error 等, 与 Config 结构完全对应.
//
// =====================================================================

template<int N>
struct PIDState {
    Eigen::Matrix<double, N, 1> target   = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> actual   = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> error    = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> p_term   = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> i_term   = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> d_term   = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> integral = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> output   = Eigen::Matrix<double, N, 1>::Zero();
};

/// @brief 串级 PID 全部环的运行时状态 (每次 update 后刷新)
struct CascadePIDState {
    PIDState<2> vel;     // Layer 1: 速度→倾角       (vx,vy → pitch,roll)
    PIDState<1> yaw;     // Layer 1b: 偏航→wz_cmd    (ωz → wz_cmd)
    PIDState<2> att;     // Layer 2: 倾角→虚拟速度    (pitch,roll → vx_cmd,vy_cmd)
    PIDState<3> wheel;   // Layer 4: 轮速→力矩       (ω1,ω2,ω3 → τ1,τ2,τ3)

    // 中间变量 (雅可比前后)
    Eigen::Vector2d target_tilt      = Eigen::Vector2d::Zero();   // [pitch_des, roll_des]
    Eigen::Vector3d virtual_vel      = Eigen::Vector3d::Zero();   // [vx_cmd, vy_cmd, wz_cmd]
    Eigen::Vector3d target_wheel_vel = Eigen::Vector3d::Zero();   // [ω1_des, ω2_des, ω3_des] = J · virtual_vel
};

// =====================================================================
//              BallbotCascadeController — 串级 PID 控制器
// =====================================================================
//
// 控制架构 (4 层串级):
//
//   cmd [vx,vy,ωz]
//       │
//   ┌───▼──────────┐  ┌─────────────┐
//   │ PID_vel (2D) │  │ PID_yaw (1D)│
//   │ vx,vy→pitch, │  │ ωz_err→ωz   │
//   │       roll   │  │       _cmd  │
//   └───┬──────────┘  └──────┬──────┘
//       │ pitch_des,         │ ωz_cmd
//       │ roll_des           │
//   ┌───▼──────────┐         │
//   │ PID_att (2D) │         │
//   │ pitch,roll   │         │
//   │  → vx,vy_cmd │         │
//   └───┬──────────┘         │
//       │ vx_cmd,vy_cmd      │
//       └────────┬───────────┘
//                │ virtual_vel = [vx_cmd, vy_cmd, ωz_cmd]
//           ┌────▼────┐
//           │ J (3×3) │  雅可比: 机身速度→轮角速度
//           └────┬────┘
//                │ ω_wheel_des [ω1,ω2,ω3]
//           ┌────▼────────┐
//           │ PID_wheel   │  轮速环: ω_err → τ
//           │    (3D)     │
//           └────┬────────┘
//                │ τ [τ1,τ2,τ3] → 电机力矩
//
// =====================================================================

class BallbotCascadeController {
public:
    // =================================================================
    //                    串级 PID 配置
    // =================================================================
    struct Config {
        // Layer 1: 速度→倾角 (2D: vx→pitch, vy→roll)
        PIDParams vel_pid;

        // Layer 1b: 偏航→偏航指令 (1D)
        PIDParams yaw_pid;

        // Layer 2: 倾角→虚拟速度 (2D: pitch→vx_cmd, roll→vy_cmd)
        PIDParams att_pid;

        // Layer 4: 轮速→力矩 (3D)
        PIDParams wheel_pid;

        // 运动学配置 (用于雅可比 J)
        BallbotKinematicConfig kinematics;
    };

    // ---- 用户可直接读写的配置与状态 ----
    Config          config;   // PID 参数 + 运动学配置
    CascadePIDState state;    // 每个环的运行时数据 (每次 update 后刷新)

    // =================================================================
    /// @brief 初始化: 计算雅可比 J, 清零所有 PID 状态
    /// @return 成功返回 true (运动学配置有效)
    // =================================================================
    bool init();

    // =================================================================
    /// @brief 串级 PID 控制更新 (每控制周期调用)
    ///
    /// @param cmd_vel      目标机身速度 [vx_des, vy_des, ωz_des]
    ///                     vx,vy: m/s (IMU系), ωz: rad/s
    /// @param vel_actual   当前机身速度 [vx, vy, ωz]  (来自传感器)
    /// @param rpy_actual   当前倾角 [roll, pitch] rad (来自传感器)
    /// @param wheel_vel    当前轮角速度 [ω1, ω2, ω3] rad/s (来自电机编码器)
    /// @param output_torque 输出电机力矩 [τ1, τ2, τ3] N·m
    /// @param dt           控制周期 (s)
    // =================================================================
    void update(const Eigen::Vector3d& cmd_vel,
                const Eigen::Vector3d& vel_actual,
                const Eigen::Vector2d& rpy_actual,
                const Eigen::Vector3d& wheel_vel,
                Eigen::Vector3d&       output_torque,
                double dt);

    /// @brief 重置所有 PID 内部积分/误差
    void reset();

private:
    // 雅可比矩阵
    Eigen::Matrix3d J_;

    // ---- PID 内部状态 ----
    // Layer 1: 速度→倾角 (2D)
    Eigen::Vector2d vel_integral_{0, 0};
    Eigen::Vector2d vel_prev_error_{0, 0};

    // Layer 1b: 偏航 (1D)
    double yaw_integral_ = 0;
    double yaw_prev_error_ = 0;

    // Layer 2: 倾角→虚拟速度 (2D)
    Eigen::Vector2d att_integral_{0, 0};
    Eigen::Vector2d att_prev_error_{0, 0};

    // Layer 4: 轮速→力矩 (3D)
    Eigen::Vector3d wheel_integral_{0, 0, 0};
    Eigen::Vector3d wheel_prev_error_{0, 0, 0};

    bool initialized_ = false;

    // =================================================================
    /// @brief N 维 PID 计算 (模板)
    /// @return PID 输出, 同时填充 state_out (若非空)
    // =================================================================
    template<int N>
    static Eigen::Matrix<double, N, 1> computePID(
        const Eigen::Matrix<double, N, 1>& target,
        const Eigen::Matrix<double, N, 1>& current,
        Eigen::Matrix<double, N, 1>& integral,
        Eigen::Matrix<double, N, 1>& prev_error,
        const PIDParams& params,
        double dt,
        PIDState<N>* state_out = nullptr);
};

// =====================================================================
// 模板实现 (header-only)
// =====================================================================

template<int N>
Eigen::Matrix<double, N, 1> BallbotCascadeController::computePID(
    const Eigen::Matrix<double, N, 1>& target,
    const Eigen::Matrix<double, N, 1>& current,
    Eigen::Matrix<double, N, 1>& integral,
    Eigen::Matrix<double, N, 1>& prev_error,
    const PIDParams& params,
    double dt,
    PIDState<N>* state_out)
{
    if (dt < 1e-10)
        return Eigen::Matrix<double, N, 1>::Zero();

    const Eigen::Matrix<double, N, 1> error = target - current;

    // 梯形积分 + 逐轴抗饱和
    integral += 0.5 * (error + prev_error) * dt;
    if (params.integral_max > 0.0) {
        for (int i = 0; i < N; ++i)
            integral[i] = std::clamp(integral[i], -params.integral_max, params.integral_max);
    }

    // 微分
    const Eigen::Matrix<double, N, 1> derivative = (error - prev_error) / dt;
    prev_error = error;

    // PID 输出
    Eigen::Matrix<double, N, 1> p_term = params.Kp * error;
    Eigen::Matrix<double, N, 1> i_term = params.Ki * integral;
    Eigen::Matrix<double, N, 1> d_term = params.Kd * derivative;
    Eigen::Matrix<double, N, 1> output = p_term + i_term + d_term;

    // 输出限幅
    if (params.output_max > 0.0) {
        for (int i = 0; i < N; ++i)
            output[i] = std::clamp(output[i], -params.output_max, params.output_max);
    }

    // 写入运行时状态 (供外部读取)
    if (state_out) {
        state_out->target   = target;
        state_out->actual   = current;
        state_out->error    = error;
        state_out->p_term   = p_term;
        state_out->i_term   = i_term;
        state_out->d_term   = d_term;
        state_out->integral = integral;
        state_out->output   = output;
    }

    return output;
}

} // namespace ballbot
