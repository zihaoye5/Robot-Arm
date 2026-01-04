#!/usr/bin/env python3
"""
使用Stable-Baselines3的TD3算法训练宇树G1机械臂（7 DoF）进行位置跟踪
"""

from locale import normalize
import numpy as np
from numpy.linalg import norm
from numpy.random import normal
import gymnasium as gym
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise # 添加高斯噪声，提升探索性
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold, BaseCallback # base就是一个框架，eval在此框架上进行，主要功能就是：每个epoch自动评估、监控记录等
from stable_baselines3.common.env_util import make_vec_env, SubprocVecEnv# 创建并行环境向量, 支持多进程(默认为dummyVecEnv单进程, 如果这个时候开启多环境, 就是串行的多环境，反而会变慢)
from stable_baselines3.common.vec_env import VecNormalize  # 对观测/奖励做归一化的向量包装器
import os # 文件保存
import time
from g1_arm_env import RobotArmEnv # <--- 1. 导入环境文件名称已更新
import argparse # 命令行参数解析，配置超参/路径。
import torch.nn as nn
import signal
import sys
from stable_baselines3.common.logger import configure


class PeriodicCheckpointCallback(BaseCallback):
    """
    定期保存模型检查点的回调函数
    """
    def __init__(self, save_freq=100000, name_prefix="td3_g1_arm", verbose=0):
        """
        参数:
        - save_freq: 保存频率（步数）
        - name_prefix: 文件名前缀
        - verbose: 是否打印信息
        """
        super(PeriodicCheckpointCallback, self).__init__(verbose)
        self.save_freq = save_freq
        self.save_path = None  # 将在训练开始时动态设置
        self.name_prefix = name_prefix
        self.last_save = 0
        
    def _on_training_start(self) -> None:
        """
        训练开始时，从logger获取实际的TensorBoard日志目录
        """
        # 从logger获取实际的日志目录路径
        if hasattr(self.model, 'logger') and self.model.logger is not None:
            # 获取TensorBoard日志的完整路径
            log_dir = self.model.logger.get_dir()
            # 构建checkpoints目录路径
            self.save_path = os.path.join(log_dir, "checkpoints")
            if self.verbose > 0:
                print(f"检查点将保存到: {self.save_path}")
        else:
            # 如果无法获取logger，使用默认路径
            self.save_path = "./models/checkpoints"
            if self.verbose > 0:
                print(f"警告: 无法获取日志目录，使用默认路径: {self.save_path}")
        
    def _on_step(self) -> bool:
        # 检查是否到了保存时间
        if (self.num_timesteps - self.last_save) >= self.save_freq: 
            self.last_save = self.num_timesteps
            
            # 如果save_path还未设置，尝试再次获取
            if self.save_path is None:
                self._on_training_start()
            
            # 创建保存目录
            os.makedirs(self.save_path, exist_ok=True)
            
            # 保存模型，文件名包含步数
            model_name = f"{self.name_prefix}_step_{self.num_timesteps}"
            model_file = os.path.join(self.save_path, model_name)
            self.model.save(model_file)
            
            # 保存归一化参数
            env = self.model.get_vec_normalize_env()
            if env is not None:
                normalize_file = os.path.join(self.save_path, f"vec_normalize_step_{self.num_timesteps}.pkl")
                env.save(normalize_file)
            
            if self.verbose > 0:
                print(f"已保存检查点: {model_file} (步数: {self.num_timesteps})")
        
        return True


class SaveVecNormalizeCallback(BaseCallback):
    """
    在保存最佳模型时同时保存VecNormalize参数的回调函数
    """
    def __init__(self, eval_callback, verbose=0):
        super(SaveVecNormalizeCallback, self).__init__(verbose)
        self.eval_callback = eval_callback
        self.best_mean_reward = -np.inf
        self.best_model_path = None  # 将在训练开始时动态设置

    def _on_training_start(self) -> None:
        """
        训练开始时，从logger获取实际的TensorBoard日志目录，并更新最佳模型保存路径
        """
        # 从logger获取实际的日志目录路径
        if hasattr(self.model, 'logger') and self.model.logger is not None:
            log_dir = self.model.logger.get_dir()
            # 更新EvalCallback的最佳模型保存路径到日志目录
            self.best_model_path = os.path.join(log_dir, "best_model")
            self.eval_callback.best_model_save_path = self.best_model_path
            if self.verbose > 0:
                print(f"最佳模型将保存到: {self.best_model_path}")
        else:
            # 如果无法获取logger，使用默认路径
            self.best_model_path = "./logs/best_model"
            if self.verbose > 0:
                print(f"警告: 无法获取日志目录，使用默认路径: {self.best_model_path}")

    def _on_step(self) -> bool:
        # 检查是否有新的最佳模型
        if self.eval_callback.best_mean_reward > self.best_mean_reward:
            self.best_mean_reward = self.eval_callback.best_mean_reward
            
            # 如果best_model_path还未设置，尝试再次获取
            if self.best_model_path is None:
                self._on_training_start()
            
            # 保存VecNormalize参数到对应的日志目录
            if self.verbose > 0:
                print("保存与最佳模型对应的VecNormalize参数")
            os.makedirs(self.best_model_path, exist_ok=True)
            self.model.get_vec_normalize_env().save(os.path.join(self.best_model_path, "vec_normalize.pkl"))
        
        return True


class TD3Loader:
    """
    负责加载或初始化 Stable-Baselines3 TD3 模型和 VecNormalize 环境的工具类。
    """

    def __init__(self, RobotArmEnv, policy_kwargs, action_noise, nu, common_hyperparams):
        """
        初始化加载器，保存所有创建模型所需的参数
        """
        self.RobotArmEnv = RobotArmEnv
        self.policy_kwargs = policy_kwargs
        self.action_noise = action_noise
        self.nu = nu
        self.hyperparams = common_hyperparams

    def _get_normalize_path(self, load_path):
        """
        根据模型路径推断 VecNormalize 文件的路径
        """
        load_dir = os.path.dirname(load_path)

        # 提取步数
        try:
            step_num = load_path.split('_step_')[-1].split('.')[0]
            normalize_file = os.path.join(load_dir, f"vec_normalize_step_{step_num}.pkl")
            return normalize_file
            # normalize_file = os.path.join(load_dir, "vec_normalize.pkl")
        except:
            # 如果路径格式不匹配，返回None
            return None

    def load_or_create(self, n_envs, load_path = None):
        """
        加载环境、归一化参数，然后加载或创建 TD3 模型。

        Args:
            n_envs (int): 并行环境数量。
            load_path (str, optional): 模型检查点路径。

        Returns:
            tuple: (model, env)
        """
        # 创建或加载 VecNormalize 环境
        if load_path:
            normalize_file = self._get_normalize_path(load_path)
        else:
            normalize_file = None
        
        if normalize_file and os.path.exists(normalize_file):
            print(f"加载归一化参数：{normalize_file}")
            env = make_vec_env(lambda:self.RobotArmEnv(), n_envs = n_envs, vec_env_cls=SubprocVecEnv) # 不写默认为dummyVecEnv, 顺序进程
            env = VecNormalize.load(normalize_file, env)
            env.reset()
        else:
            if normalize_file:
                print(f"警告：未找到归一化文件 {normalize_file}。")
            print("创建全新的归一化环境")
            env = make_vec_env(lambda:self.RobotArmEnv(), n_envs = n_envs, vec_env_cls = SubprocVecEnv)
            env = VecNormalize(env, norm_obs = True, norm_reward = True)


        # 1. 定义 TD3 构造函数所需的通用参数，从 self.hyperparams 复制
        td3_kwargs = self.hyperparams.copy()

        # 2. 覆盖日志路径（用于继续训练）
        if load_path and os.path.exists(load_path):
            try:
                # 假设路径结构为: ./logs/RUN_NAME/.../model.zip
                old_run_dir = os.path.dirname(os.path.dirname(load_path)) 
                
                if "logs" in old_run_dir:
                    run_name = os.path.basename(old_run_dir)
                    
                    # 强制设置 TensorBoard Log Path 的父目录
                    td3_kwargs['tensorboard_log'] = os.path.dirname(old_run_dir) # 例如：./logs/
                    
                    # 强制设置 tb_log_name，阻止自增
                    td3_kwargs['tb_log_name'] = run_name 
                    print(f"日志将继续记录到旧目录: {os.path.join(td3_kwargs['tensorboard_log'], run_name)}")
                
            except Exception as e:
                print(f"强制设置日志路径失败: {e}")

        
        # 加载或创建模型
        if load_path and os.path.exists(load_path):
            print(f"从checkpoint开始训练: {load_path}")
            
        
            # 使用TD3_loader 加载参数,并覆盖关键参数
            model = TD3.load(
                load_path,
                env = env,
                # 重新应用超参数，确保新环境和训练环境保持一致
                policy_kwargs = self.policy_kwargs,
                action_noise = self.action_noise,
                **td3_kwargs # 展开公共超参数
            )
            # 确保 gradient_steps 和 num_timessteps 正确设置
            model.gradient_steps = n_envs
            print(f"模型当前步数： {model.num_timesteps}")

            # 模型加载成功后，必须再次手动设置 Logger，这是阻止自增的关键！
            if 'tb_log_name' in td3_kwargs:
                log_path_base = td3_kwargs.get('tensorboard_log', './logs/')
                final_log_path = os.path.join(log_path_base, td3_kwargs['tb_log_name'])
                
                # 重新配置 Logger，使用我们解析出的旧路径
                new_logger = configure(final_log_path, ["stdout", "tensorboard"])
                model.set_logger(new_logger)
                print(f"Logger 强制设置为: {final_log_path}")

        else:
            if load_path:
                print(f"错误: 模型文件 {load_path} 不存在，创建新模型。")

            print("创建新的 TD3 模型。")
            
            # 创建新模型时，直接使用所有超参数
            model = TD3(
                "MlpPolicy",
                env,
                action_noise=self.action_noise,
                policy_kwargs=self.policy_kwargs,
                **self.hyperparams
            )
            # 确保 gradient_steps 正确设置
            model.gradient_steps = n_envs 
        
        return model, env 



def train_g1_arm(save_freq=100000, n_envs=5, num_iterations=10000000, load_path = None): # 添加保存频率参数
    """
    训练宇树G1机械臂进行位置跟踪
    
    参数:
    - save_freq: 定期保存检查点的频率（步数），默认100000步
    - load_path: 如果有想要继续的训练，则可以有输入，没有的话默认为none
    """
    print("创建机械臂环境...")
    
    # # 创建环境并使用VecNormalize进行归一化
    # 在load_or_create中已经创建了环境，不需要再创建一次
    # env = make_vec_env(lambda: RobotArmEnv(), n_envs=n_envs, vec_env_cls=SubprocVecEnv) ## n_envs是make_vec_env的参数名
    # env = VecNormalize(env, norm_obs=True, norm_reward=True)
    
    # 设置动作噪声
    # n_actions = env.action_space.shape[-1]
    n_actions = 7
    # 4. 动作噪声标准差增加到 4.0，以提高探索性（适应更大的动作空间）
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=3.0 * np.ones(n_actions))
    
    # 5. 自定义TD3神经网络结构 - 增加网络规模以应对 7 DoF 和 27 维状态
    policy_kwargs = dict(
        net_arch=dict(
            pi=[512, 512, 256],  # Actor网络结构 (从 512 增加到 1024)
            qf=[512, 512, 256]   # Critic网络结构
        ),
        activation_fn=nn.ReLU
    )
    

    # TD3 公共超参数
    common_hyperparams = dict(
        tensorboard_log="./logs/",
        device="auto",
        learning_rate=6e-5,
        buffer_size=5000000,
        learning_starts=20000,
        batch_size=256,
        tau=0.001,
        gamma=0.99,
        train_freq=(1, "step"),
        # gradient_steps 将由 Loader 根据 n_envs 覆盖，这里设置一个默认值或不设置
        gradient_steps=n_envs,  
        policy_delay=4,
        target_policy_noise=0.2,
        target_noise_clip=0.5,
        verbose=1,
    )
    # 实例化 TD3Loader
    loader = TD3Loader(
        RobotArmEnv=RobotArmEnv, 
        policy_kwargs=policy_kwargs,
        action_noise=action_noise,
        nu=n_actions,
        common_hyperparams=common_hyperparams
    )

    model, env = loader.load_or_create(n_envs = n_envs, load_path = load_path)

    # 创建评估环境和回调函数
    ## 这里的nums_env通常保持 1
    eval_env = make_vec_env(lambda: RobotArmEnv(), n_envs = 1)

    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)
    # 加载训练环境的归一化参数到评估环境中
    eval_env.obs_rms = env.obs_rms
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path= None,  # 初始路径，将在训练开始时被更新
        log_path="./logs/",
        eval_freq=5000,
        deterministic=True,
        render=False
    )
    
    
    # 定义TensorBoard日志名称（与tb_log_name保持一致）
    tb_log_name = "TD3_g1_arm_run"
    
    # 创建定期保存检查点的回调函数（路径将在训练开始时动态确定）
    periodic_checkpoint_callback = PeriodicCheckpointCallback(
        save_freq=save_freq,
        name_prefix="td3_g1_arm",
        verbose=1
    )
    
    # # 创建手动中断回调函数
    # manual_interrupt_callback = ManualInterruptCallback(verbose=1)
    
    # 创建日志目录
    os.makedirs("./logs", exist_ok=True)
    os.makedirs("./models", exist_ok=True)
    
    # 创建保存VecNormalize参数的回调函数（用于最佳模型）
    save_vec_normalize_callback = SaveVecNormalizeCallback(eval_callback, verbose=1)

    print("开始训练...")
    print(f"定期保存频率: 每 {save_freq} 步保存一次检查点")
    # print("提示: 按 Ctrl+C 可以中途停止训练并保存最后一次模型数据")
    start_time = time.time()
    
    # 训练模型，同时使用四个回调函数
    model.learn(
        total_timesteps=num_iterations, # 8. 增大总训练步数
        reset_num_timesteps = False if load_path else True,
        callback=[
            eval_callback, 
            save_vec_normalize_callback, 
            periodic_checkpoint_callback,  # 添加定期保存回调
            # manual_interrupt_callback
        ],
        tb_log_name=tb_log_name, # 9. Tensorboard 日志名称更新
        log_interval=1000
    )
    

    # 保存归一化环境和最终模型
    env.save("./models/vec_normalize.pkl")
    model.save("./models/td3_g1_arm_final") # 10. 最终模型名称更新
    
    end_time = time.time()
    print(f"训练完成，耗时: {end_time - start_time:.2f}秒")
    
    return model, env


def test_g1_arm(model_path="./models/td3_g1_arm_final", # 11. 测试函数名称及默认路径更新
                   normalize_path="./models/vec_normalize.pkl",
                   num_episodes=10):
    """
    测试训练好的模型
    """
    print("加载模型并测试...")
    
    # 创建环境
    env = make_vec_env(lambda: RobotArmEnv(render_mode="human"), n_envs=1)
    
    # 加载归一化环境
    if os.path.exists(normalize_path):
        env = VecNormalize.load(normalize_path, env)
        env.training = False
        env.norm_reward = False
    
    # 加载模型
    model = TD3.load(model_path, env=env)
    
    episode_rewards = []
    
    # 运行指定数量的回合
    for episode in range(num_episodes):
        obs = env.reset()
        total_reward = 0
        
        # 打印当前回合的目标位置
        # 正确获取多层包装环境中的目标位置
        target_pos = env.venv.envs[0].env.unwrapped.target_pos                    
        print(f"Episode {episode+1} target position: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
        
        # 运行一个episode
        for i in range(5000):
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward[0] if isinstance(reward, np.ndarray) else reward
            time.sleep(0.01) # 测试时，推理频率受到限制，约为100hz
            env.render()
            
            if done:
                print(f"Episode {episode+1} finished after {i+1} timesteps")
                print(f"Episode reward: {total_reward}")
                episode_rewards.append(total_reward)
                break    
    
    env.close()
    print(f"Average reward over {num_episodes} episodes: {np.mean(episode_rewards)}")
    return episode_rewards

 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train or test G1 arm with TD3") # 12. 描述更新
    parser.add_argument("--test", action="store_true", help="Test the trained model")
    parser.add_argument("--model-path", type=str, default="./models/td3_g1_arm_final", 
                        help="Path to the model for testing")
    parser.add_argument("--normalize-path", type=str, default="./models/vec_normalize.pkl",
                        help="Path to the normalization parameters")
    parser.add_argument("--episodes", type=int, default=10,
                        help="Number of episodes to test")
    parser.add_argument("--save-freq", type=int, default=100000,
                        help="Frequency (in timesteps) to save checkpoints during training")
    parser.add_argument("--n_envs", type = int, default=5, 
                        help="The number of the models trained in the envs at the same time")
    parser.add_argument("--num_iterations", type=int,default=10000000,
                        help="Total numbers of iteration for single training")
    parser.add_argument("--load_path", type=str,default=None,
                        help="The path for the checkpoint that you want to continue")

    args = parser.parse_args()
    
    if args.test:
        test_g1_arm(args.model_path, args.normalize_path, args.episodes) # 13. 调用新的测试函数
    else:
        train_g1_arm(save_freq=args.save_freq, n_envs=args.n_envs,num_iterations=args.num_iterations, load_path=args.load_path) # 传递保存频率参数