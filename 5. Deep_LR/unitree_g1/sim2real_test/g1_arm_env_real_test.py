import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer

class RobotArmEnv(gym.Env):
    """
    机械臂末端跟踪环境，使用MuJoCo作为物理引擎
    """
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, render_mode=None):
        """
        初始化机械臂环境
        
        参数:
        - render_mode: 渲染模式
        """
        # 1. 加载新的 MuJoCo 模型
        # self.model = mujoco.MjModel.from_xml_path("robot_arm_g1_inspire.xml")
        self.model = mujoco.MjModel.from_xml_path("g1_left_arm_inspire.xml")
        self.data = mujoco.MjData(self.model)
        
        # 获取关节数量 (应为 7)
        self.nu = self.model.nu
        self.nq = self.model.nq
        
        # 2. 设置动作空间 (扭矩) - 提高上限到 25.0 (适配 G1 关节)
        self.action_space = spaces.Box(
            low=-25.0, high=25.0, shape=(self.nu,), dtype=np.float32
        )
        
        # 3. 设置状态空间
        # 状态维度: 相对位置向量(3) + 关节角度(nu) + 关节速度(nu) + 上一时刻关节扭矩(nu) + 末端速度(3) = 3*nu + 6 维
        state_dim = 3 + 7 + 7 + 7 + 3  # nu=7 -> 27维状态
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(state_dim,), dtype=np.float32
        )
        
        # 设置目标位置 (末端执行器)
        self.target_pos = np.array([0.2, 0.2, 1.0])  # 初始目标位置 (Z轴初始位置适配 G1 基座高度)
        self.previous_distance = None
        
        # 保存上一时刻的扭矩
        self.previous_torque = np.zeros(self.nu)
        
        # 保存上一时刻的关节速度
        self.previous_joint_velocities = np.zeros(self.nu)
        
        # 初始化可视化器为None
        self.render_mode = render_mode
        self.viewer = None
                
        # 重置环境
        self.reset()

    def _get_state(self):
        """
        获取当前状态
        """
        # 获取末端执行器位置（通过名称查找）
        ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        
        # 获取末端执行器位置
        ee_pos = self.data.site_xpos[ee_site_id].copy()
        
        # 计算相对位置向量 (目标位置 - 当前末端位置)
        relative_pos = self.target_pos - ee_pos
        
        # 获取关节角度
        joint_angles = self.data.qpos[:self.nu].copy()
        
        # 获取关节速度
        joint_velocities = self.data.qvel[:self.nu].copy()
        
        # 获取上一时刻的关节扭矩
        previous_torques = self.previous_torque.copy()
        
        # 4. 使用新的末端执行器连杆名称获取末端执行器速度
        ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
        ee_vel = self.data.cvel[ee_body_id][:3].copy()
        
        # 拼接状态向量 (维度自动适应新的 self.nu)
        state = np.concatenate([
            relative_pos,          # 相对位置向量 (3,)
            joint_angles,          # 关节角度 (nu,)
            joint_velocities,      # 关节角速度 (nu,)
            previous_torques,      # 上一时刻关节扭矩 (nu,)
            ee_vel                 # 末端速度 (3,)
        ])
        
        return state.astype(np.float32)



    def reset(self, seed=None, options=None):
        """
        重置环境，将目标位置随机生成在特定范围的球壳前半球空间内。
        """
        # 继承父类的 reset 方法并设置随机种子
        super().reset(seed=seed)
        
        # 重置MuJoCo数据
        mujoco.mj_resetData(self.model, self.data)


        BASE_POS_X = 0.00
        BASE_POS_Y = 0.138
        BASE_POS_Z = 1.071
        
        # 限制参数
        MAX_ARM_LENGTH = 0.49   # 最外圈半径 (手臂最大长度)
        MIN_REACH_DIST = 0.25    # 最内圈半径 (20cm)
        
        # ----------------------------------------------------
        # 在球坐标系中随机生成目标点 (以 P_base 为原点)
        # ----------------------------------------------------
        
        # 1. 距离 (Radius, ρ): 在球壳范围 [0.25m, 0.49m] 内均匀采样
        rho = self.np_random.uniform(MIN_REACH_DIST, MAX_ARM_LENGTH)
        
        
        # phi,与y轴的夹角
        phi  = self.np_random.uniform(np.pi/3, np.pi * 2 / 3)
        # theta, 与z轴的夹角
        theta = self.np_random.uniform(np.pi/2, 4/5 * np.pi)



        # 4. 转换为相对笛卡尔坐标 (x', y', z')
        
        delta_x = rho * np.sin(phi) * np.sin(theta)
        delta_y = rho * np.cos(phi) * np.sin(theta)
        delta_z = rho * np.cos(theta)

        # 5. 目标点的绝对坐标 = 基座坐标 + 相对坐标
        self.target_pos = np.array([
            BASE_POS_X + delta_x,  # X 轴：BASE_POS_X (0.1) 加上 delta_x (>= 0)
            BASE_POS_Y + delta_y,
            BASE_POS_Z + delta_z
        ])
        
        # --- 3. 环境参数和状态重置 (保持不变) ---
        
        self.success_threshold = 0.05
        self.step_count = 0
        self.previous_distance = None
        self.min_distance = None
        
        self.previous_torque = np.zeros(self.nu) 
        self.previous_joint_velocities = np.zeros(self.nu)
        self._phase_rewards_given = set()
        
        # 获取初始状态
        state = self._get_state()
        
        return state, {}

        


    def step(self, action):
            """
            执行动作并返回结果
            """
            # --- [修改1] 保存状态用于后续计算惩罚 ---
            self.previous_torque = self.data.ctrl[:self.nu].copy()
            self.previous_joint_velocities = self.data.qvel[:self.nu].copy()
            
            # 应用动作
            action = np.clip(action, -25.0, 25.0)
            self.data.ctrl[:self.nu] = action   
            
            # 仿真一步
            mujoco.mj_step(self.model, self.data)
            self.step_count += 1
            max_steps = 3000

            # 获取状态与位置信息
            state = self._get_state()
            ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
            ee_pos = self.data.site_xpos[ee_site_id].copy()
            distance = np.linalg.norm(ee_pos - self.target_pos)
            
            reward = 0
            # 时间惩罚：保持微小，仅作为打破僵局的项
            reward -= 0.05 

            # --- [修改2] 改进的对齐奖励 (30度平滑) ---
            palm_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "inhand_base_link")
            R_EE = self.data.xmat[palm_body_id].reshape(3, 3)
            V_Desired = (self.target_pos - ee_pos) / (distance + 1e-6)
            V_Palm = R_EE[:, -1] # 掌心向量
            alignment_cos = np.dot(V_Palm, V_Desired)

            # 使用 Sigmoid 实现 30 度平滑过渡 (cos(30°) ≈ 0.866)
            # 距离越近，对齐的重要性指数级增加
            w_orient_base = 20.0
            w_distance_scaling = np.exp(-1.0 * distance)
            smooth_score = 2 * (1 / (1 + np.exp(-20 * (alignment_cos - 0.866)))) - 1
            orientation_reward = w_orient_base * smooth_score * w_distance_scaling
            reward += orientation_reward

            # --- [修改3] 碰撞逻辑优化 ---
            collision_penalty = 0.0
            collision_detected = False
            for i in range(self.data.ncon):
                contact = self.data.contact[i]
                body1_id = self.model.geom_bodyid[contact.geom1]
                body2_id = self.model.geom_bodyid[contact.geom2]
                
                # 排除自碰撞和父子级连接碰撞
                is_legal = (body1_id == body2_id or 
                            self.model.body_parentid[body1_id] == body2_id or 
                            self.model.body_parentid[body2_id] == body1_id)
                
                if not is_legal:
                    collision_detected = True
                    collision_penalty = -5000.0 # 瞬间巨大惩罚
                    break
            
            reward += collision_penalty
            if collision_detected:
                # 碰撞直接结束，不返还剩余步数奖励，防止机器人通过自杀逃避每步扣分
                return state, reward, True, False, {}

            # --- [修改4] 改进的距离奖励 (使用连续梯度而非离散阶跃) ---
            # 基础距离梯度
            improvement_reward = 0
            if self.min_distance is not None:
                if distance < self.min_distance:
                    improvement_reward = 2.0 * (self.min_distance - distance)
                    self.min_distance = distance
            else:
                self.min_distance = distance
            
            # 增加一个连续的指数距离奖励，引导机器人向 0 靠近
            reach_reward = 5.0 * np.exp(-2.0 * distance) 
            reward += (improvement_reward + reach_reward - distance * 0.5)

            # --- [修改5] 关键：靠近时的速度约束与平滑 ---
            ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
            ee_vel = self.data.cvel[ee_body_id][:3].copy()
            ee_speed = np.linalg.norm(ee_vel)
            
            # 动态速度阈值：距离越近，允许的速度越低
            # 1.0 * distance 意味着 0.1m 时限速 0.1m/s
            v_limit = max(0.05, 1.5 * distance) 
            if ee_speed > v_limit:
                reward -= 5.0 * (ee_speed - v_limit) # 超速惩罚

            # 关节平滑惩罚：惩罚加速度（抑制抖动）和能量消耗（速度平方）
            current_joint_velocities = self.data.qvel[:self.nu].copy()
            accel_penalty = -0.05 * np.sum(np.abs(current_joint_velocities - self.previous_joint_velocities))
            vel_sq_penalty = -0.001 * np.sum(np.square(current_joint_velocities))
            reward += (accel_penalty + vel_sq_penalty)

            # --- [修改6] 成功判定逻辑 ---
            done = False
            if distance <= self.success_threshold:
                done = True
                reward += 10000.0 # 成功基准奖
                reward += 2.0 * (max_steps - self.step_count) # 效率奖
                
                # 软着陆奖励：如果成功时速度极低，给予额外丰厚奖励
                if ee_speed < 0.05:
                    reward += 2000.0
                elif ee_speed < 0.1:
                    reward += 500.0

            truncated = self.step_count >= max_steps
            return state, reward, done, truncated, {}




    def render(self):
        """
        渲染环境
        """
        if self.render_mode is None:
            return
            
        # 如果可视化器尚未创建，则创建一个
        if self.viewer is None:
            try:
                # 尝试创建被动式可视化器
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                # 初始化目标可视化添加标记
                self._target_viz_added = False
            except Exception as e:
                print(f"无法启动可视化器: {e}")
                return
        
        # 如果可视化器已经创建，同步更新场景
        if self.viewer:
            try:
                # 添加目标位置的可视化（绿色小球）
                self._add_target_visualization()
                self.viewer.sync()
            except Exception as e:
                print(f"可视化器同步失败: {e}")
                self.viewer = None

    def _add_target_visualization(self):
        """
        在目标位置添加一个绿色小球用于可视化
        """
        if self.viewer:
            # 获取用户场景对象
            scn = self.viewer.user_scn
            
            # 检查是否已经添加了目标可视化
            if not getattr(self, '_target_viz_added', False):
                # 增加一个几何体（小球）
                if scn.ngeom < scn.maxgeom:
                    # 获取新增几何体的引用
                    self._target_geom = scn.ngeom
                    geom = scn.geoms[scn.ngeom]
                    scn.ngeom += 1
                    # 初始化几何体为球体
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.02, 0.02, 0.02]),  # 球体半径
                        self.target_pos,  # 球体位置（目标位置）
                        np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),  # 单位矩阵
                        np.array([0.0, 1.0, 0.0, 0.8])  # RGBA: 绿色，半透明
                    )
                    # 设置几何体类别为仅可视化，不参与物理碰撞
                    geom.category = mujoco.mjtCatBit.mjCAT_DECOR
                    # 标记已添加目标可视化
                    self._target_viz_added = True
            else:
                # 更新已存在的几何体位置
                geom = scn.geoms[self._target_geom]
                geom.pos = self.target_pos
    def close(self):
        """
        关闭可视化器
        """
        if self.viewer:
            self.viewer.close()
            self.viewer = None

# 用于sim2real测试
class G1RealRobotEnv(gym.Env):
    """
    实机部署适配器：将 DDS 数据包装成 Gym 观测空间
    """
    def __init__(self, urdf_path, target_pos=np.array([0.4, 0.1, 1.0])):
        super().__init__()
        # 1. 初始化 Pinocchio (与你提供的代码一致)
        self.model_pin = pin.buildModelFromUrdf(urdf_path)
        self.data_pin = self.model_pin.createData()
        self.ee_frame_id = self.model_pin.getFrameId("left_base_link")
        self.joint_names = [
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ]
        self.joint_ids = [self.model_pin.getJointId(n) for n in self.joint_names]
        
        # 2. DDS 订阅设置
        self.low_state = None
        self.target_pos = target_pos
        self.left_arm_index = range(15, 22)
        
        # 定义与训练时完全一致的 Space
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(27,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)

    def _get_obs(self):
        # 核心逻辑：从 low_state 提取并拼装 27 维向量
        q = np.array([self.low_state.motor_state[i].q for i in self.left_arm_index])
        dq = np.array([self.low_state.motor_state[i].dq for i in self.left_arm_index])
        tau = np.array([self.low_state.motor_state[i].tau_est for i in self.left_arm_index])
        
        # 计算 FK 获取 ee_pos 和 ee_vel
        q_full = np.zeros(self.model_pin.nq)
        dq_full = np.zeros(self.model_pin.nv)
        for i, j_id in enumerate(self.joint_ids):
            q_full[self.model_pin.joints[j_id].idx_q] = q[i]
            dq_full[self.model_pin.joints[j_id].idx_v] = dq[i]
        
        pin.forwardKinematics(self.model_pin, self.data_pin, q_full, dq_full)
        pin.updateFramePlacements(self.model_pin, self.data_pin)
        
        ee_pos = self.data_pin.oMf[self.ee_frame_id].translation.copy()
        v_spatial = pin.getFrameVelocity(self.model_pin, self.data_pin, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        ee_vel = v_spatial.linear.copy()
        rel_pos = self.target_pos - ee_pos

        # 组装 27 维：相对位置(3) + 角度(7) + 速度(7) + 扭矩(7) + 末端速度(3)
        return np.concatenate([rel_pos, q, dq, tau, ee_vel]).astype(np.float32)

    def step(self, action):
        # 这里负责将 TD3 的 action 发送给 DDS
        # 示例：send_low_cmd(action) 
        # 注意：此处需要添加你之前的 Action Scale (0.25) 和 Default Pos
        obs = self._get_obs()
        return obs, 0.0, False, False, {}

    def reset(self, seed=None, options=None):
        while self.low_state is None:
            time.sleep(0.01)
        return self._get_obs(), {}
