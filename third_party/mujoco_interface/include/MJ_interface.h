#ifndef MJ_INTERFACE_H
#define MJ_INTERFACE_H

#include "mujoco/mujoco.h"
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Dense"

class MJ_Interface {
public:
    // ---- 用户配置（在调用 initialize() 之前设置）----
    std::vector<std::string> cfg_jointNames;       // 需要监控/控制的关节名称列表
    std::string cfg_baseName;                       // 基链接名称（留空则 basePos/baseLinVel 保持为 0）
    std::string cfg_orientationSensorName;          // 四元数传感器名称（留空则 baseQuat/rpy 保持为 0）
    std::string cfg_gyroSensorName;                 // 陀螺仪传感器名称（留空则 baseAngVel 保持为 0）
    std::string cfg_accSensorName;                  // 加速度计传感器名称（留空则 baseAcc 保持为 0）
    std::string cfg_velSensorName;                  // 速度传感器名称（留空则不读取）

    // ---- 公开数据（由 updateSensorValues() 填充）----
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_pos_all;
    std::vector<double> motor_vel;
    std::vector<double> motor_vel_Old;
    std::vector<double> motor_acc;
    double rpy[3]{0};
    double yaw_simgle{0};
    int    yaw_N{0};
    Eigen::Matrix3d base_rot;
    double baseQuat[4]{0};
    double basePos[3]{0};
    double baseLinVel[3]{0};
    double baseAcc[3]{0};
    double baseAngVel[3]{0};

    // 构造函数：仅保存 mjModel/mjData 指针
    MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn);

    // 在设置完 cfg_* 配置后调用，逐项查找 ID。
    // 未配置或查找失败的项会输出警告，对应数据保持为 0。
    void initialize();

    // 从 MuJoCo 数据中更新所有已配置的传感器/关节状态
    void updateSensorValues();

    // 将扭矩向量写入 MuJoCo 控制数组
    void setMotorsTorque(std::vector<double> &tauIn);

private:
    mjModel *mj_model;
    mjData  *mj_data;

    std::vector<int> jntId_qpos;   // 关节在 qpos 中的地址偏移，-1 表示无效
    std::vector<int> jntId_qvel;   // 关节在 qvel 中的地址偏移，-1 表示无效
    std::vector<int> jntId_dctl;   // 作动器 ID，-1 表示无效

    int orientataionSensorId{-1};
    int velSensorId{-1};
    int gyroSensorId{-1};
    int accSensorId{-1};
    int baseBodyId{-1};

    double timeStep{0.001};
};

#endif
