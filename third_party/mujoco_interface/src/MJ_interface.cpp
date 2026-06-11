#include "MJ_interface.h"

MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn)
    : mj_model(mj_modelIn), mj_data(mj_dataIn)
{
    timeStep = mj_model->opt.timestep;
}

void MJ_Interface::initialize()
{
    jointNum = static_cast<int>(cfg_jointNames.size());

    // --- 关节查找 ---
    jntId_qpos.assign(jointNum, -1);
    jntId_qvel.assign(jointNum, -1);
    jntId_dctl.assign(jointNum, -1);
    motor_pos.assign(jointNum, 0);
    motor_vel.assign(jointNum, 0);
    motor_acc.assign(jointNum, 0);
    motor_pos_Old.assign(jointNum, 0);
    motor_vel_Old.assign(jointNum, 0);
    motor_pos_all.assign(jointNum, 0);

    if (jointNum == 0) {
        std::cerr << "[MJ_Interface] Warning: cfg_jointNames is empty, no joints to monitor/control."
                  << std::endl;
    }

    for (int i = 0; i < jointNum; i++) {
        int tmpId = mj_name2id(mj_model, mjOBJ_JOINT, cfg_jointNames[i].c_str());
        if (tmpId == -1) {
            std::cerr << "[MJ_Interface] Warning: joint \"" << cfg_jointNames[i]
                      << "\" not found in XML model." << std::endl;
            continue;
        }
        jntId_qpos[i] = mj_model->jnt_qposadr[tmpId];
        jntId_qvel[i] = mj_model->jnt_dofadr[tmpId];

        // 作动器名称：尝试去掉关节名中的 "_joint" 后缀，否则用关节名本身
        std::string motorName = cfg_jointNames[i];
        size_t pos = motorName.find("_joint");
        if (pos != std::string::npos) {
            motorName = motorName.substr(0, pos);
        }
        tmpId = mj_name2id(mj_model, mjOBJ_ACTUATOR, motorName.c_str());
        if (tmpId == -1) {
            std::cerr << "[MJ_Interface] Warning: actuator \"" << motorName
                      << "\" not found in XML model." << std::endl;
        } else {
            jntId_dctl[i] = tmpId;
        }
    }

    // --- 基链接 ---
    if (!cfg_baseName.empty()) {
        baseBodyId = mj_name2id(mj_model, mjOBJ_BODY, cfg_baseName.c_str());
        if (baseBodyId == -1) {
            std::cerr << "[MJ_Interface] Warning: base link \"" << cfg_baseName
                      << "\" not found in XML model, basePos/baseLinVel will be 0."
                      << std::endl;
        }
    } else {
        std::cerr << "[MJ_Interface] Warning: cfg_baseName not set, basePos/baseLinVel will be 0."
                  << std::endl;
    }

    // --- 四元数传感器 ---
    if (!cfg_orientationSensorName.empty()) {
        orientataionSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, cfg_orientationSensorName.c_str());
        if (orientataionSensorId == -1) {
            std::cerr << "[MJ_Interface] Warning: orientation sensor \"" << cfg_orientationSensorName
                      << "\" not found, baseQuat/rpy will be 0." << std::endl;
        }
    } else {
        std::cerr << "[MJ_Interface] Warning: cfg_orientationSensorName not set, baseQuat/rpy will be 0."
                  << std::endl;
    }

    // --- 陀螺仪传感器 ---
    if (!cfg_gyroSensorName.empty()) {
        gyroSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, cfg_gyroSensorName.c_str());
        if (gyroSensorId == -1) {
            std::cerr << "[MJ_Interface] Warning: gyro sensor \"" << cfg_gyroSensorName
                      << "\" not found, baseAngVel will be 0." << std::endl;
        }
    } else {
        std::cerr << "[MJ_Interface] Warning: cfg_gyroSensorName not set, baseAngVel will be 0."
                  << std::endl;
    }

    // --- 加速度计传感器 ---
    if (!cfg_accSensorName.empty()) {
        accSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, cfg_accSensorName.c_str());
        if (accSensorId == -1) {
            std::cerr << "[MJ_Interface] Warning: acc sensor \"" << cfg_accSensorName
                      << "\" not found, baseAcc will be 0." << std::endl;
        }
    } else {
        std::cerr << "[MJ_Interface] Warning: cfg_accSensorName not set, baseAcc will be 0."
                  << std::endl;
    }

    // --- 速度传感器 ---
    if (!cfg_velSensorName.empty()) {
        velSensorId = mj_name2id(mj_model, mjOBJ_SENSOR, cfg_velSensorName.c_str());
        if (velSensorId == -1) {
            std::cerr << "[MJ_Interface] Warning: vel sensor \"" << cfg_velSensorName
                      << "\" not found." << std::endl;
        }
    } else {
        std::cerr << "[MJ_Interface] Warning: cfg_velSensorName not set." << std::endl;
    }
}

void MJ_Interface::updateSensorValues()
{
    // --- 关节位置 / 速度 ---
    for (int i = 0; i < jointNum; i++) {
        if (jntId_qpos[i] < 0) continue;

        motor_pos_Old[i] = motor_pos[i];
        motor_pos[i] = mj_data->qpos[jntId_qpos[i]];

        // 计算位置变化量，考虑过零情况
        double delta_pos = motor_pos[i] - motor_pos_Old[i];

        // 过零判断：如果变化量超过π，说明发生了跨越±π边界的情况
        if (delta_pos > 3.1415926) {
            // 从正向跨越到负向（逆时针过零）
            delta_pos -= 2.0 * 3.1415926;
        } else if (delta_pos < -3.1415926) {
            // 从负向跨越到正向（顺时针过零）
            delta_pos += 2.0 * 3.1415926;
        }

        // 累加总距离
        motor_pos_all[i] += delta_pos;

        motor_vel_Old[i] = motor_vel[i];
        motor_vel[i] = mj_data->qvel[jntId_qvel[i]];
        motor_acc[i] = (motor_vel[i] - motor_vel_Old[i])/timeStep;
    }

    // --- 四元数 & 欧拉角 ---
    if (orientataionSensorId >= 0) {
        for (int i = 0; i < 4; i++) {
            baseQuat[i] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId] + i];
        }
        // MuJoCo 四元数顺序 [w, x, y, z] → ROS 顺序 [x, y, z, w]
        double tmp = baseQuat[0];
        baseQuat[0] = baseQuat[1];
        baseQuat[1] = baseQuat[2];
        baseQuat[2] = baseQuat[3];
        baseQuat[3] = tmp;

        rpy[0] = atan2(2 * (baseQuat[3] * baseQuat[0] + baseQuat[1] * baseQuat[2]),
                       1 - 2 * (baseQuat[0] * baseQuat[0] + baseQuat[1] * baseQuat[1]));
        rpy[1] = asin(2 * (baseQuat[3] * baseQuat[1] - baseQuat[0] * baseQuat[2]));
        rpy[2] = atan2(2 * (baseQuat[3] * baseQuat[2] + baseQuat[0] * baseQuat[1]),
                       1 - 2 * (baseQuat[1] * baseQuat[1] + baseQuat[2] * baseQuat[2]));

        // yaw 解缠绕
        if ((rpy[2] - yaw_simgle) > 3.1415926 * 0.5) {
            yaw_N -= 1.0;
        } else if ((rpy[2] - yaw_simgle) < -3.1415926 * 0.5) {
            yaw_N += 1.0;
        }
        yaw_simgle = rpy[2];
        rpy[2] = yaw_simgle + yaw_N * 2.0 * 3.1415926;
    }

    // --- 基链接位置 & 线速度（有限差分） ---
    if (baseBodyId >= 0) {
        for (int i = 0; i < 3; i++) {
            double posOld = basePos[i];
            basePos[i] = mj_data->xpos[3 * baseBodyId + i];
            baseLinVel[i] = (basePos[i] - posOld) / timeStep;
        }
    }

    // --- 加速度 ---
    if (accSensorId >= 0) {
        for (int i = 0; i < 3; i++) {
            baseAcc[i] = mj_data->sensordata[mj_model->sensor_adr[accSensorId] + i];
        }
    }

    // --- 角速度 ---
    if (gyroSensorId >= 0) {
        for (int i = 0; i < 3; i++) {
            baseAngVel[i] = mj_data->sensordata[mj_model->sensor_adr[gyroSensorId] + i];
        }
    }
}

void MJ_Interface::setMotorsTorque(std::vector<double> &tauIn)
{
    for (int i = 0; i < jointNum; i++) {
        if (jntId_dctl[i] >= 0) {
            mj_data->ctrl[jntId_dctl[i]] = tauIn.at(i);
        }
    }
}
