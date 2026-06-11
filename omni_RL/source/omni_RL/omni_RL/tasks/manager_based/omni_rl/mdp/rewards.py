# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)

def adaptive_tilt_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: str,
    command_name: str,
    w_base: float,
    k_decay: float,
    sigma_tilt: float
) -> torch.Tensor:
    """
    自适应权重姿态奖励：加速时放纵倾角，巡航时逼迫直立。
    
    Args:
        env: 强化学习环境实例
        asset_cfg: 机器人的资产名称 (如 "robot")
        command_name: 速度指令管理器的名称 (如 "base_velocity")
        w_base: 完美巡航(误差为0)时的最高奖励权重
        k_decay: 权重衰减系数 (决定对速度误差的宽容度)
        sigma_tilt: 倾角惩罚的敏感度 (方差的倒数)
        
    Returns:
        torch.Tensor: 形状为 (num_envs,) 的奖励张量
    
    tips:
        r_balance是倾角接近0的奖励 越接近0奖励最高为1 之后逐渐衰减
        w_tilt是根据速度误差动态调整的权重 有速度的时候允许更大的倾角 
        w_tilt * r_balance 有速度的时候降低权重 允许偏移 速度误差越小 越希望保持0倾角
    """
    # 1. 获取机器人资产
    robot: Articulation = env.scene[asset_cfg]

    # 2. 获取目标 XY 平面速度 (假设指令的最后输出前两维是 Vx, Vy)
    # shape: (num_envs, 2)
    target_vel_xy = env.command_manager.get_command(command_name)[:, :2]

    # 3. 获取当前机体线速度 (Base Frame)
    # root_lin_vel_b 表示在机体坐标系下的平动速度
    current_vel_xy = robot.data.root_lin_vel_b[:, :2]

    # 4. 计算速度误差 L2 范数: |V_cmd - V_curr|
    # shape: (num_envs,)
    vel_error = torch.norm(target_vel_xy - current_vel_xy, dim=-1)

    # 5. 计算动态权重 W_tilt: 误差越大，权重越趋近于 0
    w_tilt = w_base * torch.exp(-k_decay * vel_error)

    # 6. 计算等效倾角惩罚项
    # 利用机体系下的重力投影 projected_gravity_b
    # 取 XY 分量，其平方和等效于倾角的平方
    g_xy = robot.data.projected_gravity_b[:, :2]
    tilt_sq = torch.sum(torch.square(g_xy), dim=-1)

    # 7. 计算姿态基础得分并乘上动态权重
    r_balance = torch.exp(-sigma_tilt * tilt_sq)

    return w_tilt * r_balance
