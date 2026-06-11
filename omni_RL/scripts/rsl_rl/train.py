# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#### 核心：
#### gymnasium标准接口
#### wrapper 翻译接口对接rl算法
#### 物理蒸馏种子：确认随机性 相同的种子会出现相同的随机情况

"""Script to train RL agent with RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
### 命令行可输入参数
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
#action="store_true" 意味着在终端只要加上 --video，这个变量就变成 True，默认是 False。
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
#环境数量
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
#任务ID号
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
#超参数
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
#随机种子
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
#最大迭代次数
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
#开启多卡分布式训练开关
parser.add_argument(
    "--distributed", action="store_true", default=False, help="Run training with multiple GPUs or nodes."
)
#导入/导出空间的描述文件
parser.add_argument("--export_io_descriptors", action="store_true", default=False, help="Export IO descriptors.")
parser.add_argument(
    "--ray-proc-id", "-rid", type=int, default=None, help="Automatically configured by Ray integration, otherwise None."
)
# append RSL-RL cli arguments
### 根据参数设置rsl-rl
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
### 根据参数设置仿真平台
AppLauncher.add_app_launcher_args(parser)
### 把终端输入的参数分成两拨。认识的放进 args_cli，不认识的（通常是发给 Hydra 库处理的配置文件参数）放进 hydra_args 列表。
args_cli, hydra_args = parser.parse_known_args()

# always enable cameras to record video
### 录制video时开启画面渲染
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
### sys.argv 存放了整个命令行。为了让后文的 Hydra（配置管理库）能够正常解析它想要的参数而不报错，
### 我们手动把 sys.argv 清空，只保留脚本名 sys.argv[0] 和给 Hydra 准备的参数 hydra_args
### 对应上文的拆成两端
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
### 启动仿真平台
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


"""Check for minimum supported RSL-RL version."""

import importlib.metadata as metadata
import platform

from packaging import version

# check minimum supported rsl-rl version
### 检查rsl_rl版本
RSL_RL_VERSION = "3.0.1"
installed_version = metadata.version("rsl-rl-lib") #获取当前版本
### 版本过低则启动窗口重新安装满足需求的rsl_rl版本
if version.parse(installed_version) < version.parse(RSL_RL_VERSION):
    if platform.system() == "Windows":
        cmd = [r".\isaaclab.bat", "-p", "-m", "pip", "install", f"rsl-rl-lib=={RSL_RL_VERSION}"]
    else:
        cmd = ["./isaaclab.sh", "-p", "-m", "pip", "install", f"rsl-rl-lib=={RSL_RL_VERSION}"]
    print(
        f"Please install the correct version of RSL-RL.\nExisting version is: '{installed_version}'"
        f" and required version is: '{RSL_RL_VERSION}'.\nTo install the correct version, run:"
        f"\n\n\t{' '.join(cmd)}\n"
    )
    exit(1)

"""Rest everything follows."""
### 导入日志，操作系统，时间模块
import logging
import os
import time
from datetime import datetime
### 导入gymnasium（env和算法的接口标准）和pytorch
import gymnasium as gym
import torch
### 导入 策略蒸馏训练器和标准的PPO训练器
from rsl_rl.runners import DistillationRunner, OnPolicyRunner
### 多种环境基类
from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
### 打印字典
from isaaclab.utils.dict import print_dict
### 输出yaml工具
from isaaclab.utils.io import dump_yaml
### isaaclab对接rsl_rl的wrapper，翻译gymnasium给rsl_rl
from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
### 导入isaaclab任务
### 它会在底层触发所有的 __init__.py，把你所有的机器人环境 ID 注册到 gym 系统里
import isaaclab_tasks  # noqa: F401
### 导入寻找断点路径的工具的hydra装饰器
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

# import logger
logger = logging.getLogger(__name__)

import omni_RL.tasks  # noqa: F401
### pytorch性能设置
### 精度设置
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
### 关闭cudnn的确定性计算
torch.backends.cudnn.deterministic = False
### 关闭自动寻找最优卷积算法
torch.backends.cudnn.benchmark = False

###魔法装饰器 @hydra_task_config：它会拦截程序的运行。
# 拿着你在终端输入的 --task (比如 Go2)，去注册表里找到对应的配置类，
# 把物理配置塞进 env_cfg，把算法配置塞进 agent_cfg，然后才把它们当作参数传进 main 函数
@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Train with RSL-RL agent."""
    # override configurations with non-hydra CLI arguments
    ### 算法的参数配置写入
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    ### 读命令行的仿真环境数量 没有按照物理配置里的默认参数来
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    ### 读命令行的最大迭代次数 没有则按照算法配置里的默认参数来
    agent_cfg.max_iterations = (
        args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
    )

    # handle deprecated configurations
    ### 处理因为版本更新可能废弃的物理参数
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    ###物理引擎种子 和算法引擎种子同步
    env_cfg.seed = agent_cfg.seed
    ###运行设备：cpu/gpu
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
    # check for invalid combination of CPU device with distributed training
    ### cpu还用分布式训练--报错
    if args_cli.distributed and args_cli.device is not None and "cpu" in args_cli.device:
        raise ValueError(
            "Distributed training is not supported when using CPU device. "
            "Please use GPU device (e.g., --device cuda) for distributed training."
        )

    # multi-gpu training configuration
    ### 多显卡并行计算 我不配不看。。。
    if args_cli.distributed:
        env_cfg.sim.device = f"cuda:{app_launcher.local_rank}"
        agent_cfg.device = f"cuda:{app_launcher.local_rank}"

        # set seed to have diversity in different threads
        seed = agent_cfg.seed + app_launcher.local_rank
        env_cfg.seed = seed
        agent_cfg.seed = seed

    # specify directory for logging experiments
    ### 日志路径
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    ### 转成---绝对路径
    log_root_path = os.path.abspath(log_root_path)
    ### 打印输出绝对路径
    print(f"[INFO] Logging experiment in directory: {log_root_path}")
    # specify directory for logging runs: {time-stamp}_{run_name}
    ### 打印日志的名称（task名称/时间。。。）
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # The Ray Tune workflow extracts experiment name using the logging line below, hence, do not
    # change it (see PR #2346, comment-2819298849)
    ### 打印日志名称
    print(f"Exact experiment name requested from command line: {log_dir}")
    ### 有运行名称就加个名称
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root_path, log_dir)

    # set the IO descriptors export flag if requested
    if isinstance(env_cfg, ManagerBasedRLEnvCfg):
        env_cfg.export_io_descriptors = args_cli.export_io_descriptors
    else:
        logger.warning(
            "IO descriptors are only supported for manager based RL environments. No IO descriptors will be exported."
        )

    # set the log directory for the environment (works for all environment types)
    ### 写入日志路径信息等
    env_cfg.log_dir = log_dir

    # create isaac environment
    ### 带着上述所有的配置，在显存中生成整个世界（包括成千上万只机器狗）。如果要录制视频，就要求渲染器输出 RGB 图像阵列。
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    ### 如果是单智能体 做一下转换称rl算法需要的形式
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # save resume path before creating a new log_dir
    ### 如果是继续训练，或者是做蒸馏，去过去的日志文件夹里把之前的 .pt 权重文件路径找出来
    if agent_cfg.resume or agent_cfg.algorithm.class_name == "Distillation":
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    # wrap for video recording
    ### 视频配置 输出mp4
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "train"),
            "step_trigger": lambda step: step % args_cli.video_interval == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    ### 初始化训练启动时间
    start_time = time.time()

    # wrap around environment for rsl-rl
    ### 把原生的 Isaac Lab 环境套上 RslRlVecEnvWrapper。
    ### 这个封装类干了两件事：1. 剥开所有的字典，变成纯张量（因为 RSL-RL 只认纯张量）。 
    ### 2. 拦截网络输出的 action，做数值截断防爆处理。
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # create runner from rsl-rl
    ### 根据配置，实例化 PPO 引擎（通常是 OnPolicyRunner）。
    ### 它会在初始化时，根据环境的 observation_space 和 action_space，用 PyTorch 建立 Actor 和 Critic 神经网络。
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    # write git state to logs
    ### 在日志里记录当前代码的 Git 版本信息
    runner.add_git_repo_to_log(__file__)
    # load the checkpoint
    ### 如果前面找到了之前训练的 .pt 文件，就立刻把权重加载到刚才建好的神经网络里
    if agent_cfg.resume or agent_cfg.algorithm.class_name == "Distillation":
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        # load previously trained model
        runner.load(resume_path)

    # dump the configuration into log-directory
    ### 把这一局所有的配置详情（物理质量、各种奖励权重、学习率等）打印成 YAML 文本并永久存档，方便以后的复现和参数对比
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)

    # run training
    ### 这是整个脚本执行时间最长的一行代码。
    ### 程序进入死循环：神经网络看数据 -> 输出动作 -> 环境执行并反馈分数 -> 收集满一个 Buffer 后计算梯度 -> 更新网络权重。
    ### init_at_random_ep_len=True：一个小技巧，刚开局时给 4096 个环境设定随机的初始存活时间倒计时。
    ## 防止 4096 条狗在未来同一时刻集体死亡，导致 CPU/GPU 重置时发生可怕的算力拥堵。
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    ### 循环结束后（达到了最大训练代数），打印总耗时
    print(f"Training time: {round(time.time() - start_time, 2)} seconds")

    # close the simulator
    ### 关闭仿真平台
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
