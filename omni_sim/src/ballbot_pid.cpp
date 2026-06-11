#include "ballbot_pid.h"

#include <algorithm>
#include <cmath>

namespace ballbot {

// =====================================================================
//           createOmniBallbotConfig — omni_ballbot 默认配置
// =====================================================================
//
// 所有数值取自 omni_ballbot/xml/omni_ballbot.xml:
//
//   <body name="base_link" pos="0 0 0.333">          ← IMU 原点 (世界系)
//   <body name="balance_ball" pos="0 0 0.15">        ← 球心   (世界系)
//   <geom name="ball_geom" type="sphere" size="0.15"> ← 球半径
//
//   wheel1_base: pos="0.18364  0       -0.081395"  quat="0.939693  0         0.342019  0"
//   wheel2_base: pos="-0.091819 0.15904 -0.081395"  quat="0.469844 -0.296198  0.171009  0.813799"
//   wheel3_base: pos="-0.091819 -0.15904 -0.081395" quat="0.469844  0.296198  0.171009 -0.813799"
//
//   MuJoCo quat 格式: (w, x, y, z)
//   轮子 joint axis: (1, 0, 0) → X轴为旋转轴, Y轴为驱动方向
//
// =====================================================================

BallbotKinematicConfig createOmniBallbotConfig()
{
    BallbotKinematicConfig cfg;

    // ---- 球与机身参数 (直接取自 XML) ----
    cfg.ball_radius           = 0.15;   // ball_geom size
    cfg.imu_to_ball_center_z  = 0.183;  // base_link.z(0.333) - ball.z(0.15)

    // ---- 三轮几何参数 (取自 XML body pos / quat) ----
    cfg.wheels.resize(3);

    // 轮心在 IMU 系下的位置 (XML body pos, 相对 base_link)
    const Eigen::Vector3d wheel_pos[3] = {
        { 0.18364,  0.0,      -0.081395},  // wheel1_base
        {-0.091819,  0.15904, -0.081395},  // wheel2_base
        {-0.091819, -0.15904, -0.081395}   // wheel3_base
    };

    // 轮系→IMU系 四元数 (XML body quat, w,x,y,z 格式)
    const double wheel_quat[3][4] = {
        {0.939693,  0.0,        0.342019,  0.0},       // wheel1
        {0.469844, -0.296198,  0.171009,  0.813799},   // wheel2
        {0.469844,  0.296198,  0.171009, -0.813799}    // wheel3
    };

    // 球心在 IMU 系下的位置
    const Eigen::Vector3d ball_center_imu(0.0, 0.0, -cfg.imu_to_ball_center_z);

    double r_w_sum = 0.0;

    for (int i = 0; i < 3; ++i) {
        WheelKinematicConfig& wc = cfg.wheels[i];

        // — 旋转矩阵 R_i (四元数 → 旋转矩阵) —
        const double* q = wheel_quat[i];
        Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
        wc.rotation_wheel_to_imu = quat.toRotationMatrix();

        // — 接触点 Pi 估算 —
        // 接触点位于球面上、球心→轮心方向的交点:
        //   dir = (wheel_pos[i] - ball_center_imu).normalized()
        //   Pi  = ball_center_imu + r_b * dir
        const Eigen::Vector3d dir =
            (wheel_pos[i] - ball_center_imu).normalized();
        wc.contact_position_imu = ball_center_imu + cfg.ball_radius * dir;
        wc.ball_to_contact_imu  = wc.contact_position_imu - ball_center_imu;

        // — 有效轮半径 r_w —
        // 轮系下接触点距 X 旋转轴在 YZ 平面内的距离:
        //   P_wheel = R_iᵀ · (contact_imu - wheel_center_imu)
        //   r_w     = sqrt(P_wheel.y² + P_wheel.z²)
        const Eigen::Vector3d contact_in_wheel =
            wc.rotation_wheel_to_imu.transpose()
            * (wc.contact_position_imu - wheel_pos[i]);
        const double r_w_i = std::sqrt(
            contact_in_wheel.y() * contact_in_wheel.y() +
            contact_in_wheel.z() * contact_in_wheel.z()
        );
        r_w_sum += r_w_i;

        // active_direction_wheel 保持默认 (0, 1, 0)
    }

    cfg.wheel_radius = r_w_sum / 3.0;

    return cfg;
}

// =====================================================================
//                computeBallbotJacobian
// =====================================================================
//
// 目标: 推导 3×3 雅可比矩阵 J, 满足  ω_wheel = J · [vx, vy, ωz]ᵀ
//
// ---- 坐标系定义 ----
//
//   惯性系 (world):  固连地面
//   IMU系  (body):   固连机身, 原点在 IMU
//   球心系 (ball):   原点在球心, 各轴与 IMU 系平行, 仅 Z 向平移 -h
//   轮系   (wheel_i): 固连第 i 个全向轮
//
//   几何量:
//     p_b   = (0, 0, -h)            球心在 IMU 系下的位置
//     r_i   = contact_position_imu  IMU原点 → 接触点 Pi
//     r_bp  = ball_to_contact_imu   球心 → 接触点 Pi
//     r_b   = ball_radius           球半径
//     r_w   = wheel_radius          轮有效驱动半径
//     R_i   = rotation_wheel_to_imu 轮系→IMU系 旋转矩阵
//     d_i   = active_direction_wheel 主动驱动方向 (轮系, 恒为 Y轴)
//     d_imu = R_i · d_i             主动方向在 IMU 系下的表示
//
//   轮子 joint axis = (1,0,0) → X 轴为旋转轴, 驱动沿 Y 方向
//
//
// ---- 步骤1: 地面纯滚动约束 → 球角速度 ω_b 与机身速度的关系 ----
//
//   球心速度 (刚体运动):  v_ball = v + ω × p_b
//
//   球在地面接触点处速度为 0 (无滑动):
//     v + ω × p_b + ω_b × (0, 0, -r_b) = 0          (1)
//
//   展开两个叉乘:
//     ω × (0, 0, -h)  = [-h·ω_y,  h·ω_x,  0]ᵀ
//     ω_b × (0,0,-r_b) = [-r_b·ω_by, r_b·ω_bx, 0]ᵀ
//
//   代入 (1) 解得 ω_b 的两个分量 (第3分量 ω_bz 自由):
//     ω_bx = -(v_y + h·ω_x) / r_b                      (2a)
//     ω_by =  (v_x - h·ω_y) / r_b                      (2b)
//     v_z  =  0                                        (2c) ← 球不离地
//
//
// ---- 步骤2: 轮-球无滑约束 → 轮角速度表达式 ----
//
//   在接触点 Pi, 轮表面与球表面速度相等 (无滑动).
//   投影到主动驱动方向 d_imu 上:
//
//     轮表面在 Pi 的速度:  v + ω×r_i + r_w·φ̇_i·d_imu
//     球表面在 Pi 的速度:  v_ball + ω_b×r_bp = (v + ω×p_b) + ω_b×r_bp
//
//   投影等式 (两边同时点乘 d_imu):
//     d_imuᵀ·v + (r_i × d_imu)ᵀ·ω + r_w·φ̇_i
//   = d_imuᵀ·v + (p_b × d_imu)ᵀ·ω + (r_bp × d_imu)ᵀ·ω_b
//
//   消去 d_imuᵀ·v, 并利用 r_bp = r_i − p_b:
//     r_w·φ̇_i = (r_bp × d_imu)ᵀ·ω_b − (r_bp × d_imu)ᵀ·ω
//
//   定义 a_i = r_bp × d_imu  (第 i 个轮子的力矩方向矢量):
//     r_w·φ̇_i = a_iᵀ·(ω_b − ω)                          (3)
//
//   写成矩阵形式 (A = [a_1, a_2, a_3]):
//     r_w·φ̇ = Aᵀ·(ω_b − ω)                              (4)
//
//
// ---- 步骤3: 代回地面约束, 消去 ω_b ----
//
//   ω_b − ω = [ω_bx−ω_x,  ω_by−ω_y,  ω_bz−ω_z]ᵀ
//
//   将 (2a)(2b) 代入:
//     ω_bx − ω_x = −(v_y + h·ω_x)/r_b − ω_x
//                = −v_y/r_b − (h/r_b + 1)·ω_x
//
//     ω_by − ω_y =  (v_x − h·ω_y)/r_b − ω_y
//                =  v_x/r_b − (h/r_b + 1)·ω_y
//
//     ω_bz − ω_z =  ω_bz − ω_z
//
//   写成矩阵:  ω_b − ω = C_full · [v_x,v_y,v_z,ω_x,ω_y,ω_z]ᵀ + [0,0,ω_bz]ᵀ
//
//   其中 C_full ∈ R^(3×6):
//
//              vx      vy   vz     ωx         ωy    ωz
//           [    0, -1/r_b, 0,  -(a),         0,    0 ]  ← ω_bx−ω_x
//   C_full = [1/r_b,     0, 0,     0,       -(a),    0 ]  ← ω_by−ω_y
//           [    0,     0, 0,     0,         0,    -1 ]  ← ω_bz−ω_z
//
//   a = h/r_b + 1
//
//   → 可控自由度 = {vx, vy, ωz}, 共 3 个
//
//   提取这三列构造 C ∈ R^(3×3):
//
//         [    0, -1/r_b,  0 ]
//     C = [1/r_b,      0,  0 ]
//         [    0,      0, -1 ]
//
//   此时:  ω_b − ω = C · [vx, vy, ωz]ᵀ     (取 ω_bz = 0, 不影响差动控制)
//
//
// ---- 步骤4: 组装雅可比 ----
//
//   将上式代入 (4):
//     r_w·φ̇ = Aᵀ · C · [vx, vy, ωz]ᵀ
//
//   即:
//     φ̇ = (1/r_w) · Aᵀ · C · u
//
//   其中 u = [vx, vy, ωz]ᵀ,  J = (1/r_w) · Aᵀ · C  ∈ R^(3×3)
//
// ---- 代码实现 ----↓

Eigen::Matrix3d computeBallbotJacobian(const BallbotKinematicConfig& cfg)
{
    const double r_b = cfg.ball_radius;
    const double r_w = cfg.wheel_radius;

    // Step 1: 构造 A = [a_1, a_2, a_3] ∈ R^(3×3)
    //         a_i = r_bp × (R_i · d_i)    (力矩方向矢量)
    Eigen::Matrix3d A;
    for (int i = 0; i < 3; ++i) {
        const auto& w = cfg.wheels[static_cast<size_t>(i)];
        const Eigen::Vector3d d_imu = w.rotation_wheel_to_imu
                                    * w.active_direction_wheel;   // 轮系Y轴→IMU系
        A.col(i) = w.ball_to_contact_imu.cross(d_imu);           // r_bp × d_imu
    }

    // Step 2: 构造 C ∈ R^(3×3), 从 C_full 提取 {vx, vy, ωz} 三列
    //         [    0, -1/r_b,  0 ]
    //     C = [1/r_b,      0,  0 ]
    //         [    0,      0, -1 ]
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    C(0, 1) = -1.0 / r_b;   // ∂(ω_bx−ω_x) / ∂v_y
    C(1, 0) =  1.0 / r_b;   // ∂(ω_by−ω_y) / ∂v_x
    C(2, 2) = -1.0;          // ∂(ω_bz−ω_z) / ∂ω_z

    // Step 3: J = (1/r_w) · Aᵀ · C
    return (1.0 / r_w) * A.transpose() * C;
}

} // namespace ballbot

// =====================================================================
//           BallbotCascadeController 实现
// =====================================================================

namespace ballbot {

bool BallbotCascadeController::init()
{
    if (config.kinematics.wheels.size() != 3) {
        return false;
    }

    J_ = computeBallbotJacobian(config.kinematics);
    reset();
    initialized_ = true;
    return true;
}

void BallbotCascadeController::reset()
{
    vel_integral_.setZero();
    vel_prev_error_.setZero();
    yaw_integral_ = 0;
    yaw_prev_error_ = 0;
    att_integral_.setZero();
    att_prev_error_.setZero();
    wheel_integral_.setZero();
    wheel_prev_error_.setZero();
}

void BallbotCascadeController::update(
    const Eigen::Vector3d& cmd_vel,
    const Eigen::Vector3d& vel_actual,
    const Eigen::Vector2d& rpy_actual,
    const Eigen::Vector3d& wheel_vel,
    Eigen::Vector3d&       output_torque,
    double dt)
{
    if (!initialized_) {
        output_torque.setZero();
        return;
    }

    // ================================================================
    // Layer 1: 速度 PID → 目标倾角
    //   [vx_err, vy_err] → [pitch_des, roll_des]
    //
    //   坐标系约定 (看向坐标轴正方向, 顺时针为正):
    //     pitch>0 = 前倾 (顶朝+X)  → a_x ≈ +g·θ_pitch → pitch_des = +vel_pid(vx_err)
    //     roll>0  = 右倾 (右低左高) → a_y ≈ -g·θ_roll  → roll_des  = -vel_pid(vy_err)
    // ================================================================
    Eigen::Vector2d vel_target{cmd_vel.x(), cmd_vel.y()};
    Eigen::Vector2d vel_curr{vel_actual.x(), vel_actual.y()};
    Eigen::Vector2d tilt_raw = computePID<2>(
        vel_target, vel_curr,
        vel_integral_, vel_prev_error_,
        config.vel_pid, dt,
        &state.vel);

    state.target_tilt = Eigen::Vector2d(tilt_raw.x(), -tilt_raw.y());

    // ================================================================
    // Layer 1b: 偏航 PID → wz_cmd
    // ================================================================
    Eigen::Matrix<double, 1, 1> yaw_target{cmd_vel.z()};
    Eigen::Matrix<double, 1, 1> yaw_curr{vel_actual.z()};
    Eigen::Matrix<double, 1, 1> yaw_integral{yaw_integral_};
    Eigen::Matrix<double, 1, 1> yaw_prev_err{yaw_prev_error_};
    Eigen::Matrix<double, 1, 1> yaw_cmd_1d = computePID<1>(
        yaw_target, yaw_curr,
        yaw_integral, yaw_prev_err,
        config.yaw_pid, dt,
        &state.yaw);
    yaw_integral_ = yaw_integral(0);
    yaw_prev_error_ = yaw_prev_err(0);
    double wz_cmd = yaw_cmd_1d(0);

    // ================================================================
    // Layer 2: 倾角 PID → 虚拟机身速度
    //   [pitch_err, roll_err] → [vx_cmd, vy_cmd]
    //
    //   坐标系约定: 看向+Y顺时针为正 → pitch>0=前倾, pitch<0=后倾
    //     前倾(pitch>0): CoM前移 → 球需前移追CoM → vx_cmd>0
    //                     err = pitch_des-pitch = 0-(+)=负 → 需negate
    //     后倾(pitch<0): CoM后移 → 球需后移      → vx_cmd<0
    //                     err = 0-(-)=正 → 需negate
    //     右倾(roll>0):  CoM右移 → 球需右移(-Y) → vy_cmd<0
    //                     err = 0-(+)=负 → 不需negate (PID already gives neg)
    //   → vx_cmd = -att_pid(pitch_err),  vy_cmd = +att_pid(roll_err)
    // ================================================================
    Eigen::Vector2d rpy_aligned(rpy_actual.y(), rpy_actual.x()); // → [pitch, roll]
    Eigen::Vector2d att_raw = computePID<2>(
        state.target_tilt, rpy_aligned,
        att_integral_, att_prev_error_,
        config.att_pid, dt,
        &state.att);

    Eigen::Vector2d vcmd_xy(-att_raw.x(), att_raw.y());

    // ================================================================
    // Layer 3: 雅可比映射 — 虚拟机身速度 → 目标轮角速度
    //   ω_wheel_des = J · [vx_cmd, vy_cmd, wz_cmd]ᵀ
    //
    //   雅可比 J 由纯滚动约束推导, 不含电机驱动极性, 直接使用.
    // ================================================================
    state.virtual_vel = Eigen::Vector3d(vcmd_xy.x(), vcmd_xy.y(), wz_cmd);
    state.target_wheel_vel = J_ * state.virtual_vel;

    // ================================================================
    // Layer 4: 轮速 PID → 电机力矩
    //   τ = wheel_pid(ω_wheel_des - ω_wheel_actual)
    // ================================================================
    output_torque = computePID<3>(
        state.target_wheel_vel, wheel_vel,
        wheel_integral_, wheel_prev_error_,
        config.wheel_pid, dt,
        &state.wheel);
}

} // namespace ballbot
