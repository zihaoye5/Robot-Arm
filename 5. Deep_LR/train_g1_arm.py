#!/usr/bin/env python3
"""
使用Stable-Baselines3的TD3算法训练宇树G1机械臂（7 DoF）进行位置跟踪
"""

import numpy as np
import gymnasium as gym
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise # 添加高斯噪声，提升探索性
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold, BaseCallback # base就是一个框架，eval在此框架上进行，主要功能就是：每个epoch自动评估、监控记录等
from stable_baselines3.common.env_util import make_vec_env # 创建并行环境向量
from stable_baselines3.common.vec_env import VecNormalize  # 对观测/奖励做归一化的向量包装器
import os # 文件保存
import time
from g1_arm_env import RobotArmEnv # <--- 1. 导入环境文件名称已更新
import argparse # 命令行参数解析，配置超参/路径。
import torch.nn as nn
import signal
import sys


class SaveVecNormalizeCallback(BaseCallback):
    """
    在保存最佳模型时同时保存VecNormalize参数的回调函数
    """
    def __init__(self, eval_callback, verbose=0):
        super(SaveVecNormalizeCallback, self).__init__(verbose)
        self.eval_callback = eval_callback
        self.best_mean_reward = -np.inf

    def _on_step(self) -> bool:
        # 检查是否有新的最佳模型
        if self.eval_callback.best_mean_reward > self.best_mean_reward:
            self.best_mean_reward = self.eval_callback.best_mean_reward
            
            # 保存VecNormalize参数到logs/best_model目录
            if self.verbose > 0:
                print("保存与最佳模型对应的VecNormalize参数")
            vec_normalize_path = "./logs/best_model"
            os.makedirs(vec_normalize_path, exist_ok=True)
            self.model.get_vec_normalize_env().save(os.path.join(vec_normalize_path, "vec_normalize.pkl"))
        
        return True


class ManualInterruptCallback(BaseCallback):
    """
    允许手动中断训练并保存模型的回调函数
    """
    def __init__(self, verbose=0):
        super(ManualInterruptCallback, self).__init__(verbose)
        self.interrupted = False
        # 设置信号处理器来捕获Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        print('\n接收到中断信号，正在保存模型...')
        self.interrupted = True
        # 保存当前模型
        self.save_model()
        print('模型已保存，退出程序')
        sys.exit(0)
        
    def save_model(self):
        """
        保存当前模型和环境归一化参数
        """
        if self.model is not None:
            # 创建保存目录
            os.makedirs("./models/interrupted", exist_ok=True)
            
            # 2. 模型保存路径名称更新
            self.model.save("./models/interrupted/td3_g1_arm_interrupted")
            
            # 保存VecNormalize参数
            env = self.model.get_vec_normalize_env()
            if env is not None:
                env.save("./models/interrupted/vec_normalize.pkl")
                
            print("已保存中断时的模型和参数到 ./models/interrupted/")
        
    def _on_step(self) -> bool:
        # 如果收到中断信号，停止训练
        if self.interrupted:
            return False
        return True


def train_g1_arm(): # <--- 3. 函数名称更新
    """
    训练宇树G1机械臂进行位置跟踪
    """
    print("创建机械臂环境...")
    
    # 创建环境并使用VecNormalize进行归一化
    env = make_vec_env(lambda: RobotArmEnv(), n_envs=1)
    env = VecNormalize(env, norm_obs=True, norm_reward=True)
    
    # 设置动作噪声
    n_actions = env.action_space.shape[-1]
    # 4. 动作噪声标准差增加到 4.0，以提高探索性（适应更大的动作空间）
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=4.0 * np.ones(n_actions))
    
    # 5. 自定义TD3神经网络结构 - 增加网络规模以应对 7 DoF 和 27 维状态
    policy_kwargs = dict(
        net_arch=dict(
            pi=[1024, 512, 256],  # Actor网络结构 (从 512 增加到 1024)
            qf=[1024, 512, 256]   # Critic网络结构
        ),
        activation_fn=nn.ReLU
    )
    
    # 创建TD3模型
    model = TD3(
        "MlpPolicy",
        env,
        tensorboard_log="./logs/",
        action_noise=action_noise,
        verbose=1,
        device="auto",
        learning_rate=3e-4,
        buffer_size=5000000,  # 6. 增大经验回放缓冲区
        learning_starts=20000, # 7. 增大开始学习前的随机步数
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        policy_delay=4,
        target_policy_noise=0.2,
        target_noise_clip=0.5,
        policy_kwargs=policy_kwargs
    )
    
    # 创建评估环境和回调函数
    eval_env = make_vec_env(lambda: RobotArmEnv(), n_envs=1)

    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)
    # 加载训练环境的归一化参数到评估环境中
    eval_env.obs_rms = env.obs_rms
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./logs/best_model",
        log_path="./logs/",
        eval_freq=5000,
        deterministic=True,
        render=False
    )
    
    # 创建保存VecNormalize参数的回调函数
    save_vec_normalize_callback = SaveVecNormalizeCallback(eval_callback, verbose=1)
    
    # 创建手动中断回调函数
    manual_interrupt_callback = ManualInterruptCallback(verbose=1)
    
    # 创建日志目录
    os.makedirs("./logs", exist_ok=True)
    os.makedirs("./models", exist_ok=True)
    
    print("开始训练...")
    print("提示: 按 Ctrl+C 可以中途停止训练并保存最后一次模型数据")
    start_time = time.time()
    
    # 训练模型，同时使用三个回调函数
    model.learn(
        total_timesteps=10000000, # 8. 增大总训练步数
        callback=[eval_callback, save_vec_normalize_callback, manual_interrupt_callback],
        tb_log_name="TD3_g1_arm_run", # 9. Tensorboard 日志名称更新
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
            time.sleep(0.01)
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
    
    args = parser.parse_args()
    
    if args.test:
        test_g1_arm(args.model_path, args.normalize_path, args.episodes) # 13. 调用新的测试函数
    else:
        train_g1_arm() # 14. 调用新的训练函数