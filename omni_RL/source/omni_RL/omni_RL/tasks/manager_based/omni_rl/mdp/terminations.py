# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ball_out_of_range(
    env: ManagerBasedRLEnv,
    threshold: float,
    robot_cfg: SceneEntityCfg,
    ball_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Terminate when the ball's position deviates from the robot's body position by more than a threshold.

    Args:
        env: The environment.
        threshold: Maximum allowed distance (m) between ball and robot body in XY plane.
        robot_cfg: Scene entity configuration for the robot.
        ball_cfg: Scene entity configuration for the ball.

    Returns:
        Boolean tensor indicating which environments should be terminated.
    """
    # Extract robot and ball from scene
    robot: Articulation = env.scene[robot_cfg.name]
    ball: RigidObject = env.scene[ball_cfg.name]

    # Get robot body position (root link position in world frame)
    robot_pos = robot.data.root_pos_w  # shape: (num_envs, 3)

    # Get ball position (center of mass in world frame)
    ball_pos = ball.data.root_pos_w  # shape: (num_envs, 3)

    # Calculate XY planar distance between robot and ball
    xy_distance = torch.norm(robot_pos[:, :2] - ball_pos[:, :2], dim=1)

    # Return True if distance exceeds threshold
    return xy_distance > threshold
