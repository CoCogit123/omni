# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# tool
import math
import os
from dataclasses import MISSING
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
# 仿真平台
import isaaclab.sim as sim_utils
# env
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.manager_based_env_cfg import DefaultEventManagerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG 
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.actuators import ImplicitActuatorCfg
# event
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
# command
from isaaclab.envs.mdp import UniformVelocityCommandCfg
# action
from isaaclab.envs.mdp.actions.actions_cfg import JointVelocityActionCfg
# observation
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
# reward
from isaaclab.managers import RewardTermCfg as RewTerm
# termination
from isaaclab.managers import TerminationTermCfg as DoneTerm
# curriculumcfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm

from . import mdp

# URDF path for Robot 1034
ROBOT_USD_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "assets", "omni_ballbot", "omni_ballbot.usd")
)

'''env 
平地且没有高度传感器.
'''
@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    """机器人场景配置，包含地形、资产、光照等场景元素。"""

    ## 地形配置 -- 该地形为平地
    terrain = TerrainImporterCfg(
        ## USD prim 路径
        prim_path="/World/ground",
        ## 地形类型: 使用生成器模式
        terrain_type="plane",
        ## 地形生成器配置 (ROUGH_TERRAINS_CFG 定义粗糙地形参数)
        terrain_generator=None,
        ## 最大初始地形等级
        max_init_terrain_level=5,
        ## 碰撞组 (-1 表示默认碰撞组)
        collision_group=-1,
        ## 物理材质配置 (对齐 XML: robot_scene.xml:27 friction="1.0 0.003 0.0005")
        physics_material=sim_utils.RigidBodyMaterialCfg(
            ## 摩擦力合成模式: 相乘 (匹配 MuJoCo 元素乘法语义: 地摩擦 × 对物摩擦)
            friction_combine_mode="multiply",
            ## 弹性恢复合成模式: 取最小值
            restitution_combine_mode="min",
            ## 静摩擦系数 (MuJoCo sliding friction = 1.0)
            static_friction=1.0,
            ## 动摩擦系数
            dynamic_friction=1.0,
        ),
        ## 视觉材质配置 (MDL 材质文件)
        visual_material=sim_utils.MdlFileCfg(
            ## MDL 材质路径 (大理石瓷砖纹理)
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            ## 启用 UVW 投影
            project_uvw=True,
            ## 纹理缩放比例
            texture_scale=(0.25, 0.25),
        ),
        ## 是否启用调试可视化
        debug_vis=False,
    )
    
    ## assets -- 三轮全向轮机器人 (URDF导入 + PhysX凸包分解)
    ##
    ## 机器人结构说明:
    ##   - base_link: 主体 (1个)
    ##   - wheel{N}_base: 轮毂 (3个, N=1,2,3), 通过 wheel{N}_joint 连接到主体 (主动驱动)
    ##   - wheel{N}_link_A{1..6}: A型辊子 (每轮6个, 共18个), 被动自由滚动
    ##   - wheel{N}_link_B{1..6}: B型辊子 (每轮6个, 共18个), 被动自由滚动
    ##   共计: 39个link (1主体 + 3轮毂 + 36辊子), 39个continuous关节
    ##
    ## 物理参数对齐: XML 参考模型 (/omni_ballbot/xml/omni_ballbot.xml)
    ##   电机力矩: ctrlrange="-2.5 2.5" → effort_limit_sim=2.5
    ##   辊子关节: damping="0.001" frictionloss="0.001" armature="0.0001" → passive_rollers actuator
    ##   辊子几何: friction="0.3 0.001 0.0005" margin="0.002" → collision_props (摩擦需在USD中设置)
    ##
    ## 碰撞组策略 (对齐 MuJoCo contype/conaffinity 设计):
    ##   - self_collision=False: 禁用机器人内部自碰撞。
    ##     这意味着同一机器人的不同link之间不会相互碰撞检测。
    ##     MuJoCo 对应: roller(contype=2,conaffinity=1) 之间 2&1=0 → 不碰撞 (防止辊子间打架)
    ##     MuJoCo 对应: roller与地面(contype=1,conaffinity=1) 2&1=0 但 1&1=1 → 碰撞 (保证抓地力)
    ##   - collision_group=-1: 机器人整体使用全局碰撞组，与地形/球等所有外部物体碰撞。
    ##   - 摩擦模式: 全部使用 combine_mode="multiply"，匹配 MuJoCo 元素乘法语义
    ##     (Ball 1.0 × Roller 0.3 = 0.3, Ball 1.0 × Ground 1.0 = 1.0)
    ##   - 注意: 机器人 link 的物理材质 (含摩擦系数) 存储在 USD 文件中，不在本配置中设置。

    robot: ArticulationCfg = ArticulationCfg(
        ## -- 基础设置 --
        prim_path="/World/envs/env_.*/Robot",
        ## 碰撞组: -1 = 全局碰撞组 (与场景中所有物体碰撞，包括地形)
        collision_group=-1,

        ## ========== URDF 导入配置 ==========
        spawn=sim_utils.UsdFileCfg(
            ## USD 文件路径
            usd_path=ROBOT_USD_PATH,

            # activate_contact_sensors=True,  # 激活接触传感器

            ## ---------- 刚体属性 ----------
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,             # 启用重力
                retain_accelerations=False,        # 不保留子步间的加速度
                linear_damping=0.0,                # 线性阻尼 (留给电机/地面摩擦处理)
                angular_damping=0.0,               # 角阻尼 (留给电机/地面摩擦处理)
                max_linear_velocity=1000.0,        # 最大线速度上限 (m/s)
                max_angular_velocity=1000.0,       # 最大角速度上限 (deg/s)
                max_depenetration_velocity=1.0,    # 最大去穿透速度 (防止物体高速弹出)
            ),

            ## ---------- 碰撞属性 (对齐 XML: omni_ballbot.xml:32 roller margin/friction) ----------
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.002,              # 对齐 XML roller margin="0.002"
                torsional_patch_radius=0.001,      # 对齐 XML roller friction[1]=0.001 (扭转摩擦)
                min_torsional_patch_radius=0.0005, # 最小扭转 patch 半径
                rest_offset=0.0,                   # 静止偏移
            ),

            ## ---------- 关节根属性 ----------
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,     # 禁用自碰撞 (与 self_collision 保持一致)
                solver_position_iteration_count=4, # 位置求解器迭代次数 (越高越精确)
                solver_velocity_iteration_count=0, # 速度求解器迭代次数 (0 = 使用默认)
            ),

        ),  # end spawn

        ## ========== 初始状态 ==========
        init_state=ArticulationCfg.InitialStateCfg(
            ## 初始位置: 离地 0.335m 
            pos=(0.0, 0.0, 0.335),
            ## 关节初始角度 (所有关节默认0，此处仅显式声明)
            joint_pos={
                ".*": 0.0,  # 所有关节归零
            },
        ),

        ## ========== 驱动器分组配置 ==========
        ## 分为两组:
        ##   1. "drive_motors"  - 主动驱动电机 (3个轮毂关节)
        ##   2. "passive_rollers" - 自由滚动辊子 (36个辊子关节)
        actuators={
            ## --- 主动驱动电机组 ---
            ## 对应关节: wheel1_joint, wheel2_joint, wheel3_joint
            ## 作用: 接收控制器的位置/速度/力矩指令，驱动轮毂旋转
            "drive_motors": ImplicitActuatorCfg(
                joint_names_expr=["wheel[1-3]_joint"],  # 精确匹配3个驱动关节 (fullmatch避免误匹配辊子)
                stiffness=10.0,   # 位置刚度 (Nm/rad): 值越大跟踪越紧，过大会震荡
                damping=0.2,      # 速度阻尼 (Nm/(rad/s)): 抑制震荡，提供稳定性
                effort_limit_sim=2.5, # 力矩限制 (对齐 XML: omni_ballbot.xml:14 ctrlrange="-2.5 2.5")
                velocity_limit_sim=20.0,  # rad 速度限制
            ),

            ## --- 被动辊子组 ---
            ## 对应关节: wheel{N}_joint_{A/B}{1..6} (共36个)
            ## 作用: 辊子在接触地面时被动绕自身轴旋转，实现全向运动的"横向滑动"
            ##       它们不应由电机主动驱动，只需模拟轴承的低阻力自由滚动
            "passive_rollers": ImplicitActuatorCfg(
                joint_names_expr=["wheel[1-3]_joint_[AB][1-6]"],  # 匹配所有辊子关节
                stiffness=0.0,     # 零刚度 = 自由滚动 (无位置跟踪需求)
                damping=0.001,     # 微小阻尼: 模拟轴承润滑油粘滞力
                                   # MuJoCo参考: damping="0.001" frictionloss="0.001"
                friction=0.001,    # 静摩擦系数: 模拟轴承启动阻力
                                   # MuJoCo参考: frictionloss="0.001"
            ),
        },
    )

    ## assets -- 平衡球 (每个机器人正下方放置一个可自由滚动的球)
    ##
    ## 球体参数参考 MuJoCo 场景 (robot_scene.xml):
    ##   - 半径: 0.15m
    ##   - 质量: 1.2kg
    ##   - 位置: 环境原点上方 0.15m (机器人下方，与机器人轮子接触)
    ##   - 碰撞组: -1 (全局碰撞，与机器人和地面都能碰撞)
    ball: RigidObjectCfg = RigidObjectCfg(
        ## -- 基础设置 --
        prim_path="/World/envs/env_.*/Ball",
        ## 碰撞组: -1 = 全局碰撞组
        collision_group=-1,

        ## ========== 球体生成配置 ==========
        spawn=sim_utils.SphereCfg(
            ## 球体半径 (m)
            radius=0.15,
            ## ---------- 质量属性 ----------
            mass_props=sim_utils.MassPropertiesCfg(
                mass=0.6,  # 球体质量 0.6kg
            ),
            ## ---------- 刚体属性 ----------
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,           # 受重力影响
                linear_damping=0.0,              # 线性阻尼
                # [修改] 添加 0.005 角阻尼，等效替代 MuJoCo 中的 condim=6 时的 0.0005 滚动摩擦
                angular_damping=0.005,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            ## ---------- 碰撞属性 ----------
            collision_props=sim_utils.CollisionPropertiesCfg(
                # [修改] 对齐 MuJoCo 的 margin="0.002"
                contact_offset=0.002,
                # [新增] 模拟扭转摩擦 (MuJoCo的0.003)，物理公式: 0.003(扭转) / 1.0(滑动) = 0.003m 半径
                torsional_patch_radius=0.003, 
                min_torsional_patch_radius=0.001,
                rest_offset=0.0,        # 静止偏移
            ),
            ## ---------- 物理材质 (摩擦力) ----------
            ## 对齐 XML: robot_scene.xml:31-33 friction="1.0 0.003 0.0005" solref="0.02 1"
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,     # 静摩擦: 球与地面/轮子之间的抓地力 (MuJoCo sliding=1.0)
                dynamic_friction=1.0,    # 动摩擦 (滑动摩擦)
                ## [关键] 使用 multiply 匹配 MuJoCo 元素乘法语义: Ball(1.0) × Roller(0.3) = 0.3
                friction_combine_mode="multiply",
                ## 确保球体不乱弹
                restitution_combine_mode="min",
                ## [新增] 对齐 MuJoCo solref="0.02 1" → PhysX compliant contact
                ## solref(timeconst=0.02, dampratio=1.0) ≈ stiffness=5000, damping=100
                compliant_contact_stiffness=5000.0,
                compliant_contact_damping=100.0,
            ),
            ## ---------- 视觉材质 ----------
            ## 橙色球体，与 MuJoCo 的 rgba="0.8 0.4 0.2 1" 一致
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.8, 0.4, 0.2),  # 橙色
                roughness=0.4,
                metallic=0.0,
            ),
        ),

        ## ========== 初始状态 ==========
        ## 球在环境原点上方 0.15m (与机器人轮子接触的位置)
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.15),
            lin_vel=(0.0, 0.0, 0.0),
            ang_vel=(0.0, 0.0, 0.0),
        ),
    )

    ## sensors
    # 接触力传感器，挂载到机器人上所有 body，用于检测机器人与地面/物体的碰撞。
        # track_air_time=True: 启用腾空/着地时间追踪。传感器会计算每个 body 的：
            # current_air_time — 当前离地时间
            # current_contact_time — 当前着地时间
            # last_air_time / last_contact_time — 上一次腾空/着地的持续时长
        # 两个便捷方法：
            # compute_first_contact(dt) — 检查 body 是否在最近 dt 秒内刚落地
            # compute_first_air(dt) — 检查 body 是否在最近 dt 秒内刚离地
        # 核心数据输出 (sensor.data)：
            # net_forces_w — 各 body 受到的世界坐标系下净接触力 (num_envs, num_bodies, 3)
            # net_forces_w_history — 历史接触力 (num_envs, history_length, num_bodies, 3)
            # current_air_time / current_contact_time — 各 body 当前的腾空/着地时长
    # contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    
    ## light
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


##
# Action Manager 配置
# 速度驱动模式: 只控制3个主动驱动电机 (全向轮轮毂)
# 36个被动辊子由物理引擎自动模拟，不接收动作指令
##


@configclass
class ActionsCfg:
    """动作空间配置 —— 三轮全向轮速度驱动。

    Action 维度: 3 (wheel1_joint, wheel2_joint, wheel3_joint)
    每个动作值 ∈ [-1, 1]，经 scale 缩放后作为目标关节角速度 (rad/s) 发送到仿真器。
    """

    joint_vel = JointVelocityActionCfg(
        ## 指定动作作用于哪个场景资产 (对应 SceneCfg 中的 "robot")
        asset_name="robot",
        ## 仅匹配3个主动驱动关节，不包含辊子
        joint_names=["wheel[1-3]_joint"],
        ## 速度缩放系数: 输入[-1,1] → 输出[-20, 20] rad/s
        scale=20.0,
        ## 使用默认关节速度作为偏移量 (通常为0)
        use_default_offset=True,
        ## 保持关节名称顺序
        preserve_order=True,
    )


##
# Observation Manager 配置
# 观测空间: 仅观测机身状态 + 驱动电机速度，不观测被动辊子
##


@configclass
class ObservationsCfg:
    """观测空间配置 —— 策略组 (policy group)。

    总观测维度: 12
      - base_lin_vel:       机身线速度 (body-frame)            3维
      - base_ang_vel:       机身角速度 (body-frame)            3维
      - projected_gravity:  重力投影方向 (蕴含姿态/倾斜角度)   3维
      - drive_joint_vel:    驱动电机速度 (3个主动关节)         3维

    注意:
      - 不观测36个被动辊子的状态 (自由滚动，无控制意义)
      - 使用 projected_gravity 代替四元数: 维度更少 (3 vs 4)，且对轮式机器人姿态描述足够
    """

    @configclass
    class PolicyCfg(ObsGroup):
        """策略观测组 —— 所有观测拼接为一维张量。"""

        ## ---------- 机身状态 ----------
        ## 机身线速度 (body-frame, m/s)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)

        ## 机身角速度 (body-frame, rad/s)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)

        ## 重力投影方向 (body-frame, 无量纲单位向量)
        ## 编码了机身的 roll/pitch 倾斜姿态，比四元数更紧凑
        projected_gravity = ObsTerm(func=mdp.projected_gravity)

        ## ---------- 驱动电机状态 ----------
        ## 驱动电机关节速度 (仅3个主动驱动关节, rad/s)
        ## 使用 joint_names 过滤，避免将36个辊子速度也纳入观测
        drive_joint_vel = ObsTerm(
            func=mdp.joint_vel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["wheel[1-3]_joint"])},
        )

        ## 关闭维度拼接内的历史拍平警告 (可选)
        # concatenate_terms = True  # 默认值, 将所有观测沿最后维度拼接

    policy: PolicyCfg = PolicyCfg()

    
##
# Command Manager 配置
# 速度/角速度指令: 训练时给机器人下达目标速度/角速度指令
##


@configclass
class CommandsCfg:
    """指令配置 —— 速度和 heading 控制。

    CommandManager._prepare_terms 会遍历此 configclass 的字段,
    每个字段为一个 CommandTerm。此处仅配置 base_velocity 一个 term。
    """

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),    # 固定 10s 重采样
        rel_standing_envs=0.4,                # 40% env 静止
        rel_heading_envs=1.0,                  # 100% env 使用 heading 控制
        heading_command=True,                  # 开启 heading 模式 角度控制
        heading_control_stiffness=0.5,         # ω = 0.5 × heading_error kp*error
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.3, 0.3),
            lin_vel_y=(-0.3, 0.3),
            ang_vel_z=(-1.0, 1.0),
            heading=(-math.pi, math.pi),
        ),
    )


##
# Reward Manager 配置
# 任务奖励 + 正则化惩罚
##


@configclass
class RewardsCfg:
    """奖励配置 —— 速度跟踪奖励 + 正则化惩罚。

    RewardManager._prepare_terms 会遍历此 configclass 的字段,
    每个字段为一个 RewardTerm。
    """

    # ==================== 平衡 ====================

    #死亡惩罚（非超时）
    death = RewTerm(func=mdp.is_terminated, weight=-50.0)

    #自适应平衡倾角奖励 速度误差越小对0倾斜角度奖励越高 有速度则减小奖励允许倾斜
    adaptive_balance = RewTerm(
        func=mdp.adaptive_tilt_reward,
        weight=1.0,  # 这里的外部权重设为 1.0，因为我们在 params 里用 w_base 内部调参了
        params={
            "asset_cfg": "robot",              # scene_cfg 中定义的机器人实体名称
            "command_name": "base_velocity",   # command_cfg 中定义的速度生成器名称
            "w_base": 5.0,                     # 巡航时单步最高可得 1.0 分
            "k_decay": 2.5,                    # 衰减系数 (建议起步在 1.0 - 5.0 之间)
            "sigma_tilt": 15.0                 # 倾角敏感度 (数值越大，对倾斜越苛刻)
        }
    )

    # ==================== 跟踪 ====================

    ## xy 线速度跟踪: 指数核 exp(-||cmd_xy - vel_xy||^2 / std^2)
    ## 鼓励机器人准确跟踪 base_velocity 指令中的目标线速度
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )

    ## yaw 角速度跟踪: 指数核 exp(-(cmd_z - vel_z)^2 / std^2)
    ## 鼓励机器人准确跟踪 base_velocity 指令中的目标角速度
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )

    # ==================== 平滑 ====================

    ## 姿态角度抖动惩罚: 惩罚 roll/pitch 方向角速度 (L2 平方)
    ## 机身绕 x/y 轴晃动过大时给予负奖励，鼓励平稳运行
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.005)

    ## 平动加速度惩罚: 惩罚 加速度 (L2 平方)
    body_lin_acc_l2 = RewTerm(func=mdp.body_lin_acc_l2, weight=-0.05)

    ## 轮速度抖动惩罚: 惩罚相邻步动作变化率 (L2 平方)
    ## 动作直接映射到关节速度目标，惩罚动作变化 → 抑制轮速高频抖动
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.005)

    ## Z轴平动速度惩罚: Ballbot 应该贴地平移，任何垂直方向的弹跳或起伏都应被扣分
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.1)



##
# 终止 配置
# 任务奖励 + 正则化惩罚
##

@configclass
class TerminationsCfg:
    """终止条件配置 —— 机器人失稳/脱离/超时等自动重置。
    """
    ## 1.当 episode 超过 max_episode_length (30s) 时自动终止并重生
    time_out =  DoneTerm(func=mdp.time_out, time_out=True)

    ## 2.当机器人 roll 或 pitch 绝对值超过 45° 时自动终止并重生
    bad_orientation =  DoneTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": math.pi / 4, "asset_cfg": SceneEntityCfg("robot", body_names=["base_link"])},
    )

    ## 3.当球与机身位置偏差大于球的直径 (0.3m) 时自动终止并重生
    ##
    ## 设计依据:
    ##   正常运行时, 球应始终在机器人正下方, xy平面偏差应小于球半径
    ##   球直径 = 2 × 0.15m = 0.3m
    ##   若球与机身xy平面距离超过直径 → 机器人已脱离球体 → 失控状态
    ball_out_of_range = DoneTerm(
        func=mdp.ball_out_of_range,
        params={
            "threshold": 0.3,
            "robot_cfg": SceneEntityCfg("robot",body_names=["base_link"]),
            "ball_cfg": SceneEntityCfg("ball"),
        },
    )



##
# 完整环境配置
##


@configclass
class OmniRlEnvCfg(ManagerBasedRLEnvCfg):
    """三轮全向轮机器人 RL 环境总配置。

    使用速度驱动模式控制三轮全向轮，观测仅包含机身状态和驱动电机速度。
    """

    ## ========== 场景配置 ==========
    ## env_spacing: 各并行环境之间的间距 (米)，用于将多个环境平铺在同一个PhysX场景中
    scene: RobotSceneCfg = RobotSceneCfg(num_envs=4096, env_spacing=2.0)

    ## ========== 动作配置 ==========
    ## 速度驱动: 3维动作 → 3个驱动关节的角速度目标
    actions: ActionsCfg = ActionsCfg()

    ## ========== 观测配置 ==========
    ## 12维观测: 机身线速度(3) + 角速度(3) + 重力投影(3) + 驱动电机速度(3)
    observations: ObservationsCfg = ObservationsCfg()

    ## ========== 事件配置 ==========
    ## 使用默认事件管理器: 在 reset 时恢复场景到默认初始状态
    events: DefaultEventManagerCfg = DefaultEventManagerCfg()

    ## ========== 指令配置 ==========
    ## 速度/角速度指令: 通过 CommandsCfg → UniformVelocityCommandCfg 进行速度和 heading 控制
    commands: CommandsCfg = CommandsCfg()

    ## ========== 终止条件配置 ==========
    terminations: TerminationsCfg = TerminationsCfg()
        
    ## ========== 奖励配置 ==========
    rewards: RewardsCfg = RewardsCfg()

    ## ========== 课程学习配置 (可选) ==========
    curriculum: object | None = None

    ## ========== 时间与采样配置 ==========
    ## 控制频率倍率: policy 每 4 个物理步执行一次
    decimation: int = 4
    ## 每回合时长: 30 秒
    episode_length_s: float = 30.0