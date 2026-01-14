import numpy as np
import time
import os
import torch
#import pinocchio as pin 
import gymnasium as gym
from stable_baselines3 import TD3
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import mujoco
# 宇树 SDK 导入
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

# ==========================================
# 1. 核心实机适配类 (基于你提供的类名和逻辑)
# ==========================================
class G1RealRobotEnv(gym.Env):
    def __init__(self, xml_path, target_pos=np.array([0.4, 0.1, 1.0])):
        super().__init__()
        # --- 使用 MuJoCo 加载模型 ---
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # 定义 Gym 空间属性 (DummyVecEnv 需要)
        self.nu = 7
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(27,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-25.0, high=25.0, shape=(7,), dtype=np.float32)

        # 获取 site 和 body 的 ID，用于后续 FK 计算
        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self.ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
        
        # 数据存储
        self.low_state = None
        self.first_update = False
        self.target_pos = target_pos
        self.left_arm_index = range(15, 22) # G1 左臂电机索引

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.first_update = True

    def _get_obs(self):
        """
        使用 MuJoCo 计算正运动学并组装 27 维向量
        """
        if not self.first_update:
            return None

        # 1. 从 DDS 提取电机状态
        q = np.array([self.low_state.motor_state[i].q for i in self.left_arm_index])
        dq = np.array([self.low_state.motor_state[i].dq for i in self.left_arm_index])
        tau = np.array([self.low_state.motor_state[i].tau_est for i in self.left_arm_index])
        
        # 2. 将硬件数据同步到 MuJoCo Model 中进行 FK 计算
        self.data.qpos[:self.nu] = q
        self.data.qvel[:self.nu] = dq
        
        # 强制执行正运动学更新
        mujoco.mj_forward(self.model, self.data)
        
        # 获取当前末端执行器 (Site) 的笛卡尔空间位置
        ee_pos = self.data.site_xpos[self.ee_site_id].copy()
        
        # 获取末端执行器 (Body) 的笛卡尔空间线速度
        ee_vel = self.data.cvel[self.ee_body_id][:3].copy()
        
        # 计算相对位移
        rel_pos = self.target_pos - ee_pos

        # 3. 按照训练时的顺序拼接观察向量 (27维)
        obs = np.concatenate([
            rel_pos,          # 3
            q,                # 7
            dq,               # 7
            tau,              # 7
            ee_vel            # 3
        ]).astype(np.float32)
        
        return obs
# ==========================================
# 2. 推理与打印循环
# ==========================================
def run_inference_only():
    # 路径配置
    MODEL_PATH = "./best_model.zip"
    NORM_PATH = "./vec_normalize.pkl"
    URDF_PATH = "./g1_left_arm_inspire.xml" # 建议确保路径正确

    # 初始化 DDS 
    ChannelFactoryInitialize(0)
    
    # 实例化环境
    real_env = G1RealRobotEnv(URDF_PATH)
    
    # 订阅 rt/lowstate
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(real_env.LowStateHandler, 10)

    # 等待第一帧数据
    while not real_env.first_update:
        print("等待机器人 DDS 数据 (rt/lowstate)...", end="\r")
        time.sleep(0.5)
    print("\n[DDS 已连接] 开始实时推理打印...")

    # 加载 VecNormalize 包装环境
    # SB3 的 VecNormalize 需要在一个 DummyVecEnv 中加载
    venv = DummyVecEnv([lambda: real_env])
    if os.path.exists(NORM_PATH):
        venv = VecNormalize.load(NORM_PATH, venv)
        venv.training = False # 必须设为 False，否则它会继续更新均值方差
        venv.norm_reward = False
        print(f"[OK] 已加载归一化参数: {NORM_PATH}")

    # 加载训练好的 TD3 模型
    model = TD3.load(MODEL_PATH)
    print(f"[OK] 已加载模型: {MODEL_PATH}")

    try:
        while True:
            # 1. 获取当前观测
            raw_obs = real_env._get_obs()
            
            # 2. 推理
            if raw_obs is not None:
                # 注意：VecNormalize 需要输入是 batch 形式 (1, 27)
                norm_obs = venv.normalize_obs(raw_obs[np.newaxis, :])
                action, _ = model.predict(norm_obs, deterministic=True)

                # 3. 格式化打印
                os.system('clear') # 清屏方便观察
                print("="*60)
                print(f"--- G1 RL 推理状态 (影子模式) ---")
                print(f"目标坐标: {real_env.target_pos}")
                print(f"当前末端: { (real_env.target_pos - raw_obs[:3]).round(3) }")
                print(f"距离目标: { np.linalg.norm(raw_obs[:3]):.4f} m")
                print("-" * 60)
                print(f"关节角度 (q): {raw_obs[3:10].round(2)}")
                print(f"电机反馈力矩 (tau_est): {raw_obs[17:24].round(2)}")
                print("-" * 60)
                print(f"模型预测输出力矩 (Action):\n{action[0].round(3)}")
                print("="*60)
                print("按 Ctrl+C 停止打印")

            time.sleep(0.02) # 50Hz 刷新

    except KeyboardInterrupt:
        print("\n[退出] 打印停止")

if __name__ == "__main__":
    run_inference_only()
