#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <mujoco/mujoco.h>
#include "UI_interface.h"
#include "MJ_interface.h"
#include "PlotJuggler_interface.h"
#include "ballbot_pid.h"
#include "tunable_param_registry.h"

int main(int argc, char** argv) {
    char error[1000];

    // Load MuJoCo model
    std::string xml_path = "../omni_ballbot/xml/robot_scene.xml";
    if (argc > 1) xml_path = argv[1];

    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, 1000);
    if (!model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return 1;
    }
    mjData* data = mj_makeData(model);
    std::cout << "Model loaded (DOF: " << model->nv << ")" << std::endl;

    // ---- UI ----
    UIInterface ui(model, data);
    ui.cfg_window.title = "Omni-Wheel Ballbot — Cascade PID";
    ui.cfg_window.width = 1200;
    ui.cfg_window.height = 800;
    ui.cfg_camera.enable_tracking = true;
    ui.cfg_camera.track_body_id = 1;
    ui.cfg_camera.azimuth = 150.0;
    ui.cfg_camera.elevation = -16.0;
    ui.cfg_visualization.show_contact_point = true;
    ui.cfg_visualization.show_contact_force = true;
    ui.cfg_visualization.show_trajectory = true;      // 启用轨迹显示
    ui.cfg_visualization.trajectory_max_points = 500;  // 设置轨迹最大点数

    // ---- MJ Interface ----
    MJ_Interface mj_if(model, data);
    mj_if.cfg_jointNames = {"wheel1_joint", "wheel2_joint", "wheel3_joint"};
    mj_if.cfg_baseName = "base_link";
    mj_if.cfg_orientationSensorName = "base_orientation";
    mj_if.cfg_gyroSensorName        = "base_angvel";
    mj_if.cfg_accSensorName         = "base_acc";
    mj_if.cfg_velSensorName         = "base_linvel";
    mj_if.initialize();

    // =================================================================
    //                   串级 PID 控制器初始化 (位置控制模式)
    // =================================================================
    ballbot::BallbotCascadeController controller;
    controller.config.kinematics = ballbot::createOmniBallbotConfig();

    // 默认 PID 参数 (可在 ImGui 面板实时调节)
    // {Kp, Ki, Kd, integral_max, output_max}

    // Layer 0: 位置 → 速度 (新增位置环)
    ballbot::PIDParams pos_pid = {0.3, 0.0, 0.00, 0.5, 0.1};
    Eigen::Vector3d pos_integral = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_prev_error = Eigen::Vector3d::Zero();

    // Layer 1: 速度 → 倾角
    controller.config.vel_pid   = {0.05, 0.0, 0.0001, 0.1, 0.1};
    controller.config.yaw_pid   = {5.0, 0.0, 0.02, 0.5, 8.0};

    // Layer 2: 倾角 → 虚拟线速度
    controller.config.att_pid   = {300.0, 0.0, 3.0, 0.5, 10.0};

    // Layer 3: 轮速 → 力矩
    controller.config.wheel_pid = {0.8, 0.0, 0.0, 0.0, 2.0};

    if (!controller.init()) {
        std::cerr << "Cascade PID init failed (check kinematics config)" << std::endl;
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    

    // =================================================================
    //    雅可比 & 全链路 PID 极性验证
    //    坐标系约定: 看向坐标轴正方向, 顺时针为正
    //      pitch>0 = 前倾(+X)   roll>0 = 右倾(右低左高)
    // =================================================================
    // {
    //     auto saved_config = controller.config;
    //     double dt = 0.002;

    //     // ---------- 雅可比矩阵 ----------
    //     Eigen::Matrix3d J = ballbot::computeBallbotJacobian(controller.config.kinematics);
    //     std::cout << "\n==================== 雅可比 J (3x3) ====================\n" << J << std::endl;
    //     std::cout << "J 行列式 = " << J.determinant() << std::endl;

    //     Eigen::Vector3d vx(1,0,0), vy(0,1,0), wz(0,0,1);
    //     std::cout << "\nJ * [vx=1, 0, 0]  = [" << (J*vx).transpose()  << "]  ← 纯前进时三轮轮速" << std::endl;
    //     std::cout << "J * [0, vy=1, 0]  = [" << (J*vy).transpose()  << "]  ← 纯左移时三轮轮速" << std::endl;
    //     std::cout << "J * [0, 0, wz=1]  = [" << (J*wz).transpose()  << "]  ← 纯左转时三轮轮速" << std::endl;

    //     // ---------- PID 链路测试: 纯P, Kp=1, 限幅关闭 ----------
    //     controller.config.vel_pid   = {1.0, 0.0, 0.0, 0.0, 0.0};
    //     controller.config.yaw_pid   = {1.0, 0.0, 0.0, 0.0, 0.0};
    //     controller.config.att_pid   = {1.0, 0.0, 0.0, 0.0, 0.0};
    //     controller.config.wheel_pid = {1.0, 0.0, 0.0, 0.0, 0.0};
    //     Eigen::Vector3d torque;

    //     std::cout << "\n========== 全链路 PID 极性验证 (纯P, Kp=1) ==========" << std::endl;
    //     std::cout << "  约定: pitch>0=前倾, roll>0=右倾, 顺时针为正\n" << std::endl;

    //     // ================================================================
    //     // 测试1: 静止命令, 身体后倾 pitch=-0.1
    //     // ================================================================
    //     controller.reset();
    //     controller.update({0,0,0}, {0,0,0}, {0,-0.1}, {0,0,0}, torque, dt);
    //     std::cout << "────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试1] cmd=[0,0,0], 身体后倾 pitch=-0.1 rad" << std::endl;
    //     std::cout << "  物理: CoM后移 → 球需后移(-vx)推身体回正" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L1 vel:  输入 cmd_vel=[0,0] actual=[0,0]" << std::endl;
    //     std::cout << "           err  =" << controller.state.vel.error.transpose() << std::endl;
    //     std::cout << "           out  =" << controller.state.vel.output.transpose() << std::endl;
    //     std::cout << "           tilt_des[pitch,roll]=" << controller.state.target_tilt.transpose()
    //               << (controller.state.target_tilt.x()==0 && controller.state.target_tilt.y()==0 ? " [OK]" : " [FAIL]") << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L2 att:  输入 tilt_des=[0,0] rpy(pitch,roll)=[-0.1,0]" << std::endl;
    //     std::cout << "           err  =" << controller.state.att.error.transpose() << std::endl;
    //     std::cout << "           raw  =" << controller.state.att.output.transpose() << " (PID原始输出)" << std::endl;
    //     std::cout << "           vcmd_xy=[-raw.x, raw.y]=" << controller.state.virtual_vel.head<2>().transpose() << std::endl;
    //     double vx1 = controller.state.virtual_vel.x();
    //     std::cout << "           vx_cmd=" << vx1
    //               << (vx1 < 0 ? " [OK] 球后移(-X)→推身体前倾(+X)→回正" : " [FAIL] 应该<0") << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L3 J*v:  virtual_vel=" << controller.state.virtual_vel.transpose() << std::endl;
    //     std::cout << "           ω_des = J*v = [" << controller.state.target_wheel_vel.transpose() << "]" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L4 wheel: err =" << controller.state.wheel.error.transpose() << std::endl;
    //     std::cout << "           torque=" << torque.transpose() << std::endl;

    //     // ================================================================
    //     // 测试2: 静止命令, 身体前倾 pitch=+0.1
    //     // ================================================================
    //     controller.reset();
    //     controller.update({0,0,0}, {0,0,0}, {0,0.1}, {0,0,0}, torque, dt);
    //     std::cout << "\n────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试2] cmd=[0,0,0], 身体前倾 pitch=+0.1 rad" << std::endl;
    //     std::cout << "  物理: CoM前移 → 球需前移(+vx)推身体回正" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L1 vel:  tilt_des=" << controller.state.target_tilt.transpose() << std::endl;
    //     std::cout << "  L2 att:  err=" << controller.state.att.error.transpose() << std::endl;
    //     std::cout << "           raw=" << controller.state.att.output.transpose() << std::endl;
    //     std::cout << "           vcmd_xy=" << controller.state.virtual_vel.head<2>().transpose() << std::endl;
    //     double vx2 = controller.state.virtual_vel.x();
    //     std::cout << "           vx_cmd=" << vx2
    //               << (vx2 > 0 ? " [OK] 球前移(+X)→推身体后仰(-X)→回正" : " [FAIL] 应该>0") << std::endl;
    //     std::cout << "  L3 J*v:  ω_des=" << controller.state.target_wheel_vel.transpose() << std::endl;
    //     std::cout << "  L4 wheel: torque=" << torque.transpose() << std::endl;

    //     // ================================================================
    //     // 测试3: 静止命令, 右倾 roll=+0.1
    //     // ================================================================
    //     controller.reset();
    //     controller.update({0,0,0}, {0,0,0}, {0.1,0}, {0,0,0}, torque, dt);
    //     std::cout << "\n────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试3] cmd=[0,0,0], 右倾 roll=+0.1 rad (右低左高)" << std::endl;
    //     std::cout << "  物理: CoM右移(-Y) → 球需右移(-vy)推身体回正" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L1 vel:  tilt_des=" << controller.state.target_tilt.transpose() << std::endl;
    //     std::cout << "  L2 att:  err=" << controller.state.att.error.transpose() << std::endl;
    //     std::cout << "           raw=" << controller.state.att.output.transpose() << std::endl;
    //     std::cout << "           vcmd_xy=" << controller.state.virtual_vel.head<2>().transpose() << std::endl;
    //     double vy3 = controller.state.virtual_vel.y();
    //     std::cout << "           vy_cmd=" << vy3
    //               << (vy3 < 0 ? " [OK] 球右移(-Y)→推身体左倾(+Y)→回正" : " [FAIL] 应该<0") << std::endl;
    //     std::cout << "  L3 J*v:  ω_des=" << controller.state.target_wheel_vel.transpose() << std::endl;
    //     std::cout << "  L4 wheel: torque=" << torque.transpose() << std::endl;

    //     // ================================================================
    //     // 测试4: 静止命令, 左倾 roll=-0.1
    //     // ================================================================
    //     controller.reset();
    //     controller.update({0,0,0}, {0,0,0}, {-0.1,0}, {0,0,0}, torque, dt);
    //     std::cout << "\n────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试4] cmd=[0,0,0], 左倾 roll=-0.1 rad (左低右高)" << std::endl;
    //     std::cout << "  物理: CoM左移(+Y) → 球需左移(+vy)推身体回正" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L2 att:  err=" << controller.state.att.error.transpose() << std::endl;
    //     std::cout << "           raw=" << controller.state.att.output.transpose() << std::endl;
    //     std::cout << "           vcmd_xy=" << controller.state.virtual_vel.head<2>().transpose() << std::endl;
    //     double vy4 = controller.state.virtual_vel.y();
    //     std::cout << "           vy_cmd=" << vy4
    //               << (vy4 > 0 ? " [OK] 球左移(+Y)→推身体右倾(-Y)→回正" : " [FAIL] 应该>0") << std::endl;
    //     std::cout << "  L3 J*v:  ω_des=" << controller.state.target_wheel_vel.transpose() << std::endl;
    //     std::cout << "  L4 wheel: torque=" << torque.transpose() << std::endl;

    //     // ================================================================
    //     // 测试5: 前进命令 vx=0.3, 身体直立
    //     // ================================================================
    //     controller.reset();
    //     controller.update({0.3,0,0}, {0,0,0}, {0,0}, {0,0,0}, torque, dt);
    //     std::cout << "\n────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试5] cmd_vx=+0.3 m/s, 直立" << std::endl;
    //     std::cout << "  物理: 要前进 → 需前倾(正pitch)利用重力分量加速" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L1 vel:  err=" << controller.state.vel.error.transpose() << std::endl;
    //     std::cout << "           out=" << controller.state.vel.output.transpose() << std::endl;
    //     std::cout << "           tilt_des=" << controller.state.target_tilt.transpose() << std::endl;
    //     double p5 = controller.state.target_tilt.x();
    //     std::cout << "           pitch_des=" << p5
    //               << (p5 > 0 ? " [OK] 正=前倾→加速前进" : " [FAIL] 应该>0") << std::endl;
    //     std::cout << "  L2 att:  err=" << controller.state.att.error.transpose() << std::endl;
    //     std::cout << "           vcmd_xy=" << controller.state.virtual_vel.head<2>().transpose() << std::endl;
    //     std::cout << "  L3 J*v:  ω_des=" << controller.state.target_wheel_vel.transpose() << std::endl;
    //     std::cout << "  L4 wheel: torque=" << torque.transpose() << std::endl;

    //     // ================================================================
    //     // 测试6: 左移命令 vy=0.3, 直立
    //     // ================================================================
    //     controller.reset();
    //     controller.update({0,0.3,0}, {0,0,0}, {0,0}, {0,0,0}, torque, dt);
    //     std::cout << "\n────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试6] cmd_vy=+0.3 m/s, 直立" << std::endl;
    //     std::cout << "  物理: 要左移(+Y) → 需左倾(负roll)利用重力分量加速" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L1 vel:  err=" << controller.state.vel.error.transpose() << std::endl;
    //     std::cout << "           out=" << controller.state.vel.output.transpose() << std::endl;
    //     std::cout << "           tilt_des=" << controller.state.target_tilt.transpose() << std::endl;
    //     double r6 = controller.state.target_tilt.y();
    //     std::cout << "           roll_des=" << r6
    //               << (r6 < 0 ? " [OK] 负=左倾→加速左移" : " [FAIL] 应该<0") << std::endl;
    //     std::cout << "  L2 att:  err=" << controller.state.att.error.transpose() << std::endl;
    //     std::cout << "           vcmd_xy=" << controller.state.virtual_vel.head<2>().transpose() << std::endl;
    //     std::cout << "  L3 J*v:  ω_des=" << controller.state.target_wheel_vel.transpose() << std::endl;
    //     std::cout << "  L4 wheel: torque=" << torque.transpose() << std::endl;

    //     // ================================================================
    //     // 测试7: yaw 命令
    //     // ================================================================
    //     controller.reset();
    //     controller.update({0,0,0.5}, {0,0,0}, {0,0}, {0,0,0}, torque, dt);
    //     std::cout << "\n────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试7] cmd_wz=+0.5 rad/s, 直立" << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "  L1b yaw: err=" << controller.state.yaw.error(0)
    //               << " out=" << controller.state.yaw.output(0) << std::endl;
    //     std::cout << "  virtual_vel=" << controller.state.virtual_vel.transpose() << std::endl;
    //     std::cout << "  L3 J*v:  ω_des=" << controller.state.target_wheel_vel.transpose() << std::endl;
    //     std::cout << "  L4 wheel: torque=" << torque.transpose() << std::endl;

    //     // ================================================================
    //     // 测试8: 手动验证 J * virtual_vel == target_wheel_vel
    //     // ================================================================
    //     std::cout << "\n────────────────────────────────────────────────────────" << std::endl;
    //     std::cout << "[测试8] J*virtual_vel vs target_wheel_vel 一致性检查" << std::endl;
    //     Eigen::Vector3d jv = J * controller.state.virtual_vel;
    //     std::cout << "  J * virtual_vel = [" << jv.transpose() << "]" << std::endl;
    //     std::cout << "  target_wheel_vel = [" << controller.state.target_wheel_vel.transpose() << "]" << std::endl;
    //     std::cout << "  差值 = [" << (jv - controller.state.target_wheel_vel).transpose() << "]"
    //               << ((jv - controller.state.target_wheel_vel).norm() < 1e-9 ? " [OK] 一致" : " [FAIL] 不一致!") << std::endl;

    //     // 恢复
    //     controller.config = saved_config;
    //     controller.reset();
    //     std::cout << "\n========================================================\n" << std::endl;
    // }

    // ---- 命令位置和速度 (键盘 / ImGui 可调) ----
    double cmd_pos_x  = 0.0;  // 目标位置 x (m)
    double cmd_pos_y  = 0.0;  // 目标位置 y (m)
    double cmd_yaw    = 0.0;  // 目标偏航角 (rad)

    double cmd_vel_x  = 0.0;  // 通过位置环计算得到
    double cmd_vel_y  = 0.0;  // 通过位置环计算得到
    double cmd_vel_wz = 0.0;  // 通过位置环计算得到

    // =================================================================
    //                   圆形轨迹跟踪参数
    // =================================================================
    const double circle_radius = 0.5;      // 圆的半径 (m)
    const double circle_duration = 40.0;   // 完成一圈的时间 (s)
    const bool enable_circle_tracking = true;  // 是否启用圆形轨迹跟踪

    // =================================================================
    //                   可调参数注册 (ImGui 面板)
    // =================================================================
    TunableParamRegistry tunable_params;

    // 目标位置命令（可在 ImGui 中设置）
    tunable_params.registerDouble("cmd_pos_x",  &cmd_pos_x);
    tunable_params.registerDouble("cmd_pos_y",  &cmd_pos_y);
    tunable_params.registerDouble("cmd_yaw",    &cmd_yaw);

    // 位置环 PID (Layer 0: pos → vel)
    tunable_params.registerDouble("pos_Kp", &pos_pid.Kp);
    tunable_params.registerDouble("pos_Ki", &pos_pid.Ki);
    tunable_params.registerDouble("pos_Kd", &pos_pid.Kd);
    tunable_params.registerDouble("pos_integral_max", &pos_pid.integral_max);
    tunable_params.registerDouble("pos_output_max",   &pos_pid.output_max);

    // 速度环 PID (Layer 1: vel → tilt)
    tunable_params.registerDouble("vel_Kp", &controller.config.vel_pid.Kp);
    tunable_params.registerDouble("vel_Ki", &controller.config.vel_pid.Ki);
    tunable_params.registerDouble("vel_Kd", &controller.config.vel_pid.Kd);
    tunable_params.registerDouble("vel_integral_max", &controller.config.vel_pid.integral_max);
    tunable_params.registerDouble("vel_output_max",   &controller.config.vel_pid.output_max);

    // 偏航环 PID (Layer 1b: yaw → wz_cmd)
    tunable_params.registerDouble("yaw_Kp", &controller.config.yaw_pid.Kp);
    tunable_params.registerDouble("yaw_Ki", &controller.config.yaw_pid.Ki);
    tunable_params.registerDouble("yaw_Kd", &controller.config.yaw_pid.Kd);
    tunable_params.registerDouble("yaw_integral_max", &controller.config.yaw_pid.integral_max);
    tunable_params.registerDouble("yaw_output_max",   &controller.config.yaw_pid.output_max);

    // 姿态环 PID (Layer 2: tilt → virtual vel)
    tunable_params.registerDouble("att_Kp", &controller.config.att_pid.Kp);
    tunable_params.registerDouble("att_Ki", &controller.config.att_pid.Ki);
    tunable_params.registerDouble("att_Kd", &controller.config.att_pid.Kd);
    tunable_params.registerDouble("att_integral_max", &controller.config.att_pid.integral_max);
    tunable_params.registerDouble("att_output_max",   &controller.config.att_pid.output_max);

    // 轮速环 PID (Layer 4: wheel vel → torque)
    tunable_params.registerDouble("wheel_Kp", &controller.config.wheel_pid.Kp);
    tunable_params.registerDouble("wheel_Ki", &controller.config.wheel_pid.Ki);
    tunable_params.registerDouble("wheel_Kd", &controller.config.wheel_pid.Kd);
    tunable_params.registerDouble("wheel_integral_max", &controller.config.wheel_pid.integral_max);
    tunable_params.registerDouble("wheel_output_max",   &controller.config.wheel_pid.output_max);

    // 仿真倍速
    double sim_time_multiplier = 1.0;
    tunable_params.registerDouble("sim_time_multiplier", &sim_time_multiplier);

    ui.setTunableParamRegistry(&tunable_params);
    ui.cfg_imgui.enable_imgui = true;

    if (!ui.initialize()) {
        std::cerr << "Failed to initialize UI" << std::endl;
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    // =================================================================
    //                   正运动学相关变量
    // =================================================================
    // 计算雅可比逆矩阵用于正运动学: [vx, vy, wz] = J^-1 * [w1, w2, w3]
    Eigen::Matrix3d J = ballbot::computeBallbotJacobian(controller.config.kinematics);
    Eigen::Matrix3d J_inv = J.inverse();

    // 机体位置和速度 (从正运动学计算)
    Eigen::Vector3d fk_body_pos = Eigen::Vector3d::Zero();      // [x, y, yaw]
    Eigen::Vector3d fk_body_vel = Eigen::Vector3d::Zero();      // [vx, vy, wz]

    // =================================================================
    //                   PlotJuggler 调试数据流
    // =================================================================
    PlotJugglerInterface pj_if;
    pj_if.cfg_udp_host = "127.0.0.1";
    pj_if.cfg_udp_port = 9870;
    pj_if.cfg_rate_limit_hz = 0;
    pj_if.cfg_auto_timestamp = false;
    pj_if.cfg_packet_prefix = "bot.";

    // -- 机器人状态 --
    pj_if.addVariable("rpy.roll");
    pj_if.addVariable("rpy.pitch");
    pj_if.addVariable("rpy.yaw");
    pj_if.addVariable("ang_vel.x");
    pj_if.addVariable("ang_vel.y");
    pj_if.addVariable("ang_vel.z");
    pj_if.addVariable("lin_vel_body.x");
    pj_if.addVariable("lin_vel_body.y");
    pj_if.addVariable("lin_vel_body.z");
    pj_if.addVariable("motor_vel.0");
    pj_if.addVariable("motor_vel.1");
    pj_if.addVariable("motor_vel.2");
    pj_if.addVariable("motor_pos_all.0");
    pj_if.addVariable("motor_pos_all.1");
    pj_if.addVariable("motor_pos_all.2");
    pj_if.addVariable("motor_torque.0");
    pj_if.addVariable("motor_torque.1");
    pj_if.addVariable("motor_torque.2");

    // -- 正运动学计算结果 --
    pj_if.addVariable("fk.body_pos.x");
    pj_if.addVariable("fk.body_pos.y");
    pj_if.addVariable("fk.body_pos.yaw");
    pj_if.addVariable("fk.body_vel.vx");
    pj_if.addVariable("fk.body_vel.vy");
    pj_if.addVariable("fk.body_vel.wz");

    // -- 仿真直接获取的位置和速度 --
    pj_if.addVariable("sim.body_pos.x");
    pj_if.addVariable("sim.body_pos.y");
    pj_if.addVariable("sim.body_pos.yaw");
    pj_if.addVariable("sim.body_vel.vx");
    pj_if.addVariable("sim.body_vel.vy");
    pj_if.addVariable("sim.body_vel.wz");

    // -- 正运动学与仿真的差异 --
    pj_if.addVariable("fk_diff.pos_x");
    pj_if.addVariable("fk_diff.pos_y");
    pj_if.addVariable("fk_diff.yaw");
    pj_if.addVariable("fk_diff.vel_vx");
    pj_if.addVariable("fk_diff.vel_vy");
    pj_if.addVariable("fk_diff.vel_wz");

    // -- 位置环相关 --
    pj_if.addVariable("pid.pos.error_x");
    pj_if.addVariable("pid.pos.error_y");
    pj_if.addVariable("pid.pos.error_yaw");
    pj_if.addVariable("pid.pos.output_vx");
    pj_if.addVariable("pid.pos.output_vy");
    pj_if.addVariable("pid.pos.output_wz");

    // -- 命令 --
    pj_if.addVariable("cmd.pos_x");
    pj_if.addVariable("cmd.pos_y");
    pj_if.addVariable("cmd.yaw");
    pj_if.addVariable("cmd.vel_x");
    pj_if.addVariable("cmd.vel_y");
    pj_if.addVariable("cmd.vel_wz");

    // -- 串级 PID 中间变量 --
    pj_if.addVariable("pid.tilt_des.pitch");
    pj_if.addVariable("pid.tilt_des.roll");
    pj_if.addVariable("pid.virtual_vel.vx");
    pj_if.addVariable("pid.virtual_vel.vy");
    pj_if.addVariable("pid.virtual_vel.wz");
    pj_if.addVariable("pid.wheel_des.0");
    pj_if.addVariable("pid.wheel_des.1");
    pj_if.addVariable("pid.wheel_des.2");

    // -- 各环误差 & 输出 --
    pj_if.addVariable("pid.vel.error_x");
    pj_if.addVariable("pid.vel.error_y");
    pj_if.addVariable("pid.vel.output_x");
    pj_if.addVariable("pid.vel.output_y");
    pj_if.addVariable("pid.yaw.error");
    pj_if.addVariable("pid.yaw.output");
    pj_if.addVariable("pid.att.error_pitch");
    pj_if.addVariable("pid.att.error_roll");
    pj_if.addVariable("pid.att.output_x");
    pj_if.addVariable("pid.att.output_y");
    pj_if.addVariable("pid.wheel.error_0");
    pj_if.addVariable("pid.wheel.error_1");
    pj_if.addVariable("pid.wheel.error_2");
    pj_if.addVariable("pid.wheel.output_0");
    pj_if.addVariable("pid.wheel.output_1");
    pj_if.addVariable("pid.wheel.output_2");

    pj_if.initialize();

    // =================================================================
    //                   主循环
    // =================================================================
    std::cout << "\n=== Omni-Wheel Ballbot — 位置控制模式 (圆形轨迹跟踪) ===" << std::endl;
    if (enable_circle_tracking) {
        std::cout << "  [自动模式] 机器人将沿圆形轨迹运动" << std::endl;
        std::cout << "  圆半径: " << circle_radius << " m" << std::endl;
        std::cout << "  周期: " << circle_duration << " s" << std::endl;
    } else {
        std::cout << "  W/S       — 增加/减少目标位置 Y (+0.5m/-0.5m)" << std::endl;
        std::cout << "  A/D       — 增加/减少目标位置 X (+0.5m/-0.5m)" << std::endl;
        std::cout << "  Q/E       — 增加/减少目标偏航角 (+30°/-30°)" << std::endl;
    }
    std::cout << "  Backspace — 重置仿真" << std::endl;
    std::cout << "  ImGui     — 实时调节 PID 参数\n" << std::endl;

    auto start_real_time = std::chrono::high_resolution_clock::now();
    double start_sim_time = data->time;

    // 用于控制台打印频率
    double last_print_time = 0.0;
    const double print_interval = 1.0;  // 每1秒打印一次

    while (!ui.shouldClose()) {
        // ---- 时间同步 ----
        auto current_real_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> real_elapsed = current_real_time - start_real_time;
        double target_sim_time = start_sim_time + real_elapsed.count() * sim_time_multiplier;

        if (!ui.cfg_simulation.is_running) {
            start_real_time = std::chrono::high_resolution_clock::now();
            start_sim_time = data->time;
        }

        // ---- 键盘 → 目标位置命令 (直接轮询 GLFW 按键状态) ----
        GLFWwindow* window = ui.getWindow();

        // 检测UI复位：当检测到Backspace键被按下时，重置所有累积数据
        static bool last_backspace_state = false;
        bool current_backspace_state = (glfwGetKey(window, GLFW_KEY_BACKSPACE) == GLFW_PRESS);

        if (current_backspace_state && !last_backspace_state) {
            // 检测到Backspace按下的上升沿，重置正运动学累积数据
            fk_body_pos.setZero();
            pos_integral.setZero();
            pos_prev_error.setZero();
            controller.reset();

            // 同时重置目标位置为原点
            cmd_pos_x = 0.0;
            cmd_pos_y = 0.0;
            cmd_yaw = 0.0;

            // 重置仿真时间
            start_real_time = std::chrono::high_resolution_clock::now();
            start_sim_time = data->time;

            std::cout << "\n[复位] 仿真和正运动学数据已重置\n" << std::endl;
        }
        last_backspace_state = current_backspace_state;

        // =================================================================
        // 圆形轨迹生成：根据仿真时间生成目标位置
        // =================================================================
        if (enable_circle_tracking) {
            double t = data->time;  // 当前仿真时间
            double omega = 2.0 * M_PI / circle_duration;  // 角频率

            // 圆形轨迹参数方程
            cmd_pos_x = circle_radius * std::cos(omega * t);
            cmd_pos_y = circle_radius * std::sin(omega * t);
            cmd_yaw = 0.0;  // 保持yaw为0，不旋转
        } else {
            // 手动键盘控制模式（已注释Space键功能）
            // 按键修改目标位置 (增量式)
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) cmd_pos_y += 0.01;  // 向前
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) cmd_pos_y -= 0.01;  // 向后
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) cmd_pos_x += 0.01;  // 向右
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) cmd_pos_x -= 0.01;  // 向左
            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) cmd_yaw   += 0.01;  // 左转
            if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) cmd_yaw   -= 0.01;  // 右转
        }

        // ---- 物理步进 ----
        while (data->time < target_sim_time && ui.cfg_simulation.is_running) {
            double dt = model->opt.timestep;

            // 更新传感器
            mj_if.updateSensorValues();

            // ================================================================
            // 正运动学计算: 根据电机速度和yaw角计算机体位置
            // ================================================================

            // 1. 通过逆雅可比计算机体速度: [vx, vy, wz] = J^-1 * [w1, w2, w3]
            Eigen::Vector3d wheel_vel_vec(mj_if.motor_vel[0],
                                          mj_if.motor_vel[1],
                                          mj_if.motor_vel[2]);
            fk_body_vel = J_inv * wheel_vel_vec;

            // 2. 使用IMU的yaw角（而不是积分角速度）作为当前朝向
            //    这样可以利用IMU提供的准确姿态信息
            double current_yaw = mj_if.rpy[2];  // 直接使用IMU的yaw
            fk_body_pos(2) = current_yaw;

            // 3. 将机体系速度转换到世界系，然后积分得到位置
            double cos_yaw = std::cos(current_yaw);
            double sin_yaw = std::sin(current_yaw);

            // 世界系速度 = R(yaw) * 机体系速度
            double world_vx = cos_yaw * fk_body_vel(0) - sin_yaw * fk_body_vel(1);
            double world_vy = sin_yaw * fk_body_vel(0) + cos_yaw * fk_body_vel(1);

            // 积分得到世界系位置（这是从电机编码器+IMU姿态计算的）
            fk_body_pos(0) += world_vx * dt;  // x
            fk_body_pos(1) += world_vy * dt;  // y

            // ================================================================
            // Layer 0: 位置环 PID — 目标位置 → 目标速度
            // ================================================================

            // 位置误差（世界系）
            Eigen::Vector3d pos_target(cmd_pos_x, cmd_pos_y, cmd_yaw);
            Eigen::Vector3d pos_error = pos_target - fk_body_pos;

            // 梯形积分 + 抗饱和
            pos_integral += 0.5 * (pos_error + pos_prev_error) * dt;
            if (pos_pid.integral_max > 0.0) {
                for (int i = 0; i < 3; ++i)
                    pos_integral[i] = std::clamp(pos_integral[i], -pos_pid.integral_max, pos_pid.integral_max);
            }

            // 微分
            Eigen::Vector3d pos_derivative = (pos_error - pos_prev_error) / dt;
            pos_prev_error = pos_error;

            // PID 输出 (世界系速度)
            Eigen::Vector3d vel_cmd_world = pos_pid.Kp * pos_error +
                                            pos_pid.Ki * pos_integral +
                                            pos_pid.Kd * pos_derivative;

            // 输出限幅
            if (pos_pid.output_max > 0.0) {
                for (int i = 0; i < 3; ++i)
                    vel_cmd_world[i] = std::clamp(vel_cmd_world[i], -pos_pid.output_max, pos_pid.output_max);
            }

            // 将世界系速度转换到机体系
            cmd_vel_x  = cos_yaw * vel_cmd_world(0) + sin_yaw * vel_cmd_world(1);
            cmd_vel_y  = -sin_yaw * vel_cmd_world(0) + cos_yaw * vel_cmd_world(1);
            cmd_vel_wz = vel_cmd_world(2);

            // ================================================================
            // Layer 1-4: 原有的级联 PID 控制器
            // ================================================================

            // ---- 构造控制器输入 ----
            // 机体线速度 (世界系 → 机体系)
            Eigen::Quaterniond q_body(
                mj_if.baseQuat[3],   // w (ROS order: x,y,z,w)
                mj_if.baseQuat[0],   // x
                mj_if.baseQuat[1],   // y
                mj_if.baseQuat[2]    // z
            );
            Eigen::Matrix3d R_body_to_world = q_body.toRotationMatrix();
            Eigen::Vector3d lin_vel_world(mj_if.baseLinVel[0],
                                          mj_if.baseLinVel[1],
                                          mj_if.baseLinVel[2]);
            Eigen::Vector3d lin_vel_body = R_body_to_world.transpose() * lin_vel_world;

            // vel_actual = [vx_body, vy_body, ωz_body]
            Eigen::Vector3d vel_actual(lin_vel_body.x(),
                                       lin_vel_body.y(),
                                       mj_if.baseAngVel[2]);

            // rpy_actual = [roll, pitch]
            Eigen::Vector2d rpy_actual(mj_if.rpy[0], mj_if.rpy[1]);

            // wheel_vel = [ω1, ω2, ω3]
            Eigen::Vector3d wheel_vel(mj_if.motor_vel[0],
                                      mj_if.motor_vel[1],
                                      mj_if.motor_vel[2]);

            // cmd_vel = [vx_des, vy_des, ωz_des]
            Eigen::Vector3d cmd_vel(cmd_vel_x, cmd_vel_y, cmd_vel_wz);

            // ---- 串级 PID 更新 ----
            Eigen::Vector3d torque;
            controller.update(cmd_vel, vel_actual, rpy_actual, wheel_vel, torque, dt);

            // 3. 获取仿真直接提供的机体位置和速度 (用于对比)
            Eigen::Vector3d sim_body_pos(mj_if.basePos[0],
                                         mj_if.basePos[1],
                                         mj_if.rpy[2]);  // yaw
            Eigen::Vector3d sim_body_vel(lin_vel_body.x(),
                                         lin_vel_body.y(),
                                         mj_if.baseAngVel[2]);

            // 施加力矩
            std::vector<double> tau_vec = {torque.x(), torque.y(), torque.z()};
            mj_if.setMotorsTorque(tau_vec);

            // ---- PlotJuggler 发送 ----
            pj_if.setTimestamp(data->time);

            // 机器人状态
            pj_if.setValue("rpy.roll",  mj_if.rpy[0]);
            pj_if.setValue("rpy.pitch", mj_if.rpy[1]);
            pj_if.setValue("rpy.yaw",   mj_if.rpy[2]);
            pj_if.setValue("ang_vel.x", mj_if.baseAngVel[0]);
            pj_if.setValue("ang_vel.y", mj_if.baseAngVel[1]);
            pj_if.setValue("ang_vel.z", mj_if.baseAngVel[2]);
            pj_if.setValue("lin_vel_body.x", lin_vel_body.x());
            pj_if.setValue("lin_vel_body.y", lin_vel_body.y());
            pj_if.setValue("lin_vel_body.z", lin_vel_body.z());
            pj_if.setValue("motor_vel.0", mj_if.motor_vel[0]);
            pj_if.setValue("motor_vel.1", mj_if.motor_vel[1]);
            pj_if.setValue("motor_vel.2", mj_if.motor_vel[2]);
            pj_if.setValue("motor_pos_all.0", mj_if.motor_pos_all[0]);
            pj_if.setValue("motor_pos_all.1", mj_if.motor_pos_all[1]);
            pj_if.setValue("motor_pos_all.2", mj_if.motor_pos_all[2]);
            pj_if.setValue("motor_torque.0", torque.x());
            pj_if.setValue("motor_torque.1", torque.y());
            pj_if.setValue("motor_torque.2", torque.z());

            // 正运动学计算结果
            pj_if.setValue("fk.body_pos.x", fk_body_pos(0));
            pj_if.setValue("fk.body_pos.y", fk_body_pos(1));
            pj_if.setValue("fk.body_pos.yaw", fk_body_pos(2));
            pj_if.setValue("fk.body_vel.vx", fk_body_vel(0));
            pj_if.setValue("fk.body_vel.vy", fk_body_vel(1));
            pj_if.setValue("fk.body_vel.wz", fk_body_vel(2));

            // 仿真直接获取的位置和速度
            pj_if.setValue("sim.body_pos.x", sim_body_pos(0));
            pj_if.setValue("sim.body_pos.y", sim_body_pos(1));
            pj_if.setValue("sim.body_pos.yaw", sim_body_pos(2));
            pj_if.setValue("sim.body_vel.vx", sim_body_vel(0));
            pj_if.setValue("sim.body_vel.vy", sim_body_vel(1));
            pj_if.setValue("sim.body_vel.wz", sim_body_vel(2));

            // 正运动学与仿真的差异
            pj_if.setValue("fk_diff.pos_x", fk_body_pos(0) - sim_body_pos(0));
            pj_if.setValue("fk_diff.pos_y", fk_body_pos(1) - sim_body_pos(1));
            pj_if.setValue("fk_diff.yaw", fk_body_pos(2) - sim_body_pos(2));
            pj_if.setValue("fk_diff.vel_vx", fk_body_vel(0) - sim_body_vel(0));
            pj_if.setValue("fk_diff.vel_vy", fk_body_vel(1) - sim_body_vel(1));
            pj_if.setValue("fk_diff.vel_wz", fk_body_vel(2) - sim_body_vel(2));

            // ================================================================
            // 控制台打印: 定期显示正运动学与仿真的对比
            // ================================================================
            if (data->time - last_print_time >= print_interval) {
                std::cout << "\n======== 位置控制状态 (t=" << data->time << "s) ========" << std::endl;

                std::cout << "\n[目标位置]" << std::endl;
                std::cout << "  cmd_pos:  x=" << cmd_pos_x
                          << "  y=" << cmd_pos_y
                          << "  yaw=" << cmd_yaw << std::endl;

                std::cout << "\n[FK位置 vs 仿真位置]" << std::endl;
                std::cout << "  FK:   x=" << fk_body_pos(0)
                          << "  y=" << fk_body_pos(1)
                          << "  yaw=" << fk_body_pos(2) << std::endl;
                std::cout << "  SIM:  x=" << sim_body_pos(0)
                          << "  y=" << sim_body_pos(1)
                          << "  yaw=" << sim_body_pos(2) << std::endl;
                std::cout << "  Err:  Δx=" << pos_error(0)
                          << "  Δy=" << pos_error(1)
                          << "  Δyaw=" << pos_error(2) << std::endl;

                std::cout << "\n[速度命令 (位置PID输出)]" << std::endl;
                std::cout << "  cmd_vel:  vx=" << cmd_vel_x
                          << "  vy=" << cmd_vel_y
                          << "  wz=" << cmd_vel_wz << std::endl;

                std::cout << "\n[FK速度 vs 仿真速度]" << std::endl;
                std::cout << "  FK:   vx=" << fk_body_vel(0)
                          << "  vy=" << fk_body_vel(1)
                          << "  wz=" << fk_body_vel(2) << std::endl;
                std::cout << "  SIM:  vx=" << sim_body_vel(0)
                          << "  vy=" << sim_body_vel(1)
                          << "  wz=" << sim_body_vel(2) << std::endl;

                std::cout << "======================================================\n" << std::endl;

                last_print_time = data->time;
            }

            // 位置环数据
            pj_if.setValue("pid.pos.error_x",    pos_error(0));
            pj_if.setValue("pid.pos.error_y",    pos_error(1));
            pj_if.setValue("pid.pos.error_yaw",  pos_error(2));
            pj_if.setValue("pid.pos.output_vx",  cmd_vel_x);
            pj_if.setValue("pid.pos.output_vy",  cmd_vel_y);
            pj_if.setValue("pid.pos.output_wz",  cmd_vel_wz);

            // 目标位置命令
            pj_if.setValue("cmd.pos_x",  cmd_pos_x);
            pj_if.setValue("cmd.pos_y",  cmd_pos_y);
            pj_if.setValue("cmd.yaw",    cmd_yaw);

            // 命令速度
            pj_if.setValue("cmd.vel_x",  cmd_vel_x);
            pj_if.setValue("cmd.vel_y",  cmd_vel_y);
            pj_if.setValue("cmd.vel_wz", cmd_vel_wz);

            // 串级 PID 中间变量
            pj_if.setValue("pid.tilt_des.pitch",  controller.state.target_tilt.x());
            pj_if.setValue("pid.tilt_des.roll",   controller.state.target_tilt.y());
            pj_if.setValue("pid.virtual_vel.vx",  controller.state.virtual_vel.x());
            pj_if.setValue("pid.virtual_vel.vy",  controller.state.virtual_vel.y());
            pj_if.setValue("pid.virtual_vel.wz",  controller.state.virtual_vel.z());
            pj_if.setValue("pid.wheel_des.0",     controller.state.target_wheel_vel.x());
            pj_if.setValue("pid.wheel_des.1",     controller.state.target_wheel_vel.y());
            pj_if.setValue("pid.wheel_des.2",     controller.state.target_wheel_vel.z());

            // 各环误差 & 输出
            pj_if.setValue("pid.vel.error_x",    controller.state.vel.error.x());
            pj_if.setValue("pid.vel.error_y",    controller.state.vel.error.y());
            pj_if.setValue("pid.vel.output_x",   controller.state.vel.output.x());
            pj_if.setValue("pid.vel.output_y",   controller.state.vel.output.y());
            pj_if.setValue("pid.yaw.error",      controller.state.yaw.error(0));
            pj_if.setValue("pid.yaw.output",     controller.state.yaw.output(0));
            pj_if.setValue("pid.att.error_pitch", controller.state.att.error.x());
            pj_if.setValue("pid.att.error_roll",  controller.state.att.error.y());
            pj_if.setValue("pid.att.output_x",   controller.state.att.output.x());
            pj_if.setValue("pid.att.output_y",   controller.state.att.output.y());
            pj_if.setValue("pid.wheel.error_0",  controller.state.wheel.error.x());
            pj_if.setValue("pid.wheel.error_1",  controller.state.wheel.error.y());
            pj_if.setValue("pid.wheel.error_2",  controller.state.wheel.error.z());
            pj_if.setValue("pid.wheel.output_0", controller.state.wheel.output.x());
            pj_if.setValue("pid.wheel.output_1", controller.state.wheel.output.y());
            pj_if.setValue("pid.wheel.output_2", controller.state.wheel.output.z());

            pj_if.update();

            // MuJoCo 步进
            mj_step(model, data);
        }

        // UI 更新
        ui.update();

        // 帧率控制
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Cleanup
    pj_if.close();
    ui.close();
    mj_deleteData(data);
    mj_deleteModel(model);

    return 0;
}
