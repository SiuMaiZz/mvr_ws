import numpy as np
import torch
import pandas as pd
import csv
import ast

class RLCommandManualInput:
    def __init__(self):
        # 读取 CSV 数据
        csv_file_path = 'src/mvr_robot_control/data/episode_data_Dec09_12-00-16.csv'
        self.data = pd.read_csv(csv_file_path)
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.jit.load('/home/robot007/mvr_ws/src/mvr_robot_control/model/policy_1_right_arm_pitch_high_v3.pt')  # 你需要加载模型
        self.model.to(self.device)

        self.count = 0
        
        # 初始化 CSV 文件并写入表头
        self.output_file = open('src/mvr_robot_control/data/generated_output.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.output_file)
        self.csv_writer.writerow(['step', 'phase', 'obs', 'action_raw', 'action_clipped', 'action_scaled'])

    def safe_eval(self, value):
        """ 安全地解析字符串为 Python 对象 """
        try:
            # 使用 ast.literal_eval 解析列表或其他字面量对象
            return np.array(ast.literal_eval(value), dtype=np.float32)
        except (ValueError, SyntaxError):
            # 如果解析失败，返回一个默认的空数组或零数组
            print(f"Warning: Unable to parse value: {value}")
            return np.zeros(0, dtype=np.float32)

    def get_manual_input(self):
        # 从 CSV 中获取一行数据
        if self.count >= len(self.data):
            return None  # 超过数据范围，返回 None
        
        row = self.data.iloc[self.count]
        
        phase = row['phase']
        sin_pos = row['obs_sin']
        cos_pos = row['obs_cos']
        
        # 使用安全的 eval 方法处理 `obs_dof_pos` 和其他字段
        dof_pos = self.safe_eval(row['obs_dof_pos'])
        dof_vel = self.safe_eval(row['obs_dof_vel'])
        actions = self.safe_eval(row['obs_dof_actions'])

        # 处理观测输入（维度：sin_pos, cos_pos, dof_pos, dof_vel, actions）
        obs = np.concatenate([
            np.array([sin_pos, cos_pos], dtype=np.float32),  # 2 个值
            dof_pos,  # 1 个关节位置
            dof_vel,  # 1 个关节速度
            actions   # 1 个动作
        ])
        
        # 打印观测数据的形状，确保其正确
        print(f"Input shape: {obs.shape}")
        return torch.FloatTensor(obs).to(self.device).unsqueeze(0)

    def compute_action(self):
        # 获取手动输入的观测数据
        obs_tensor = self.get_manual_input()
        if obs_tensor is None:
            return None
        
        # 获取模型输出
        with torch.no_grad(): 
            action_raw = self.model(obs_tensor)

        # 处理动作输出
        action_raw = action_raw.cpu().numpy().flatten().astype(np.float32)
        
        # clip actions
        clip_actions = 18
        action_clipped = np.clip(action_raw, -clip_actions, clip_actions)
        
        # scale actions
        action_scale = 0.25
        action_scaled = action_clipped * action_scale
        
        # 获取当前帧的 phase 和 obs
        phase = self.data.iloc[self.count]['phase']
        obs = self.get_manual_input().cpu().numpy().flatten().astype(np.float32)
        
        # 将数据写入 CSV 文件
        self.csv_writer.writerow([self.count, phase, obs, action_raw[0], action_clipped[0], action_scaled[0]])

        # 打印输出以验证
        print(f"Step {self.count}: Phase {phase}, Action Raw {action_raw}, Action Clipped {action_clipped}, Action Scaled {action_scaled}")

        self.count += 1
        return action_raw, action_clipped, action_scaled

# 使用示例
if __name__ == '__main__':
    rl_command = RLCommandManualInput()
    while True:
        result = rl_command.compute_action()
        if result is None:
            break
        # 在此处理动作，发布到机器人，或用于其他目的

    # 关闭文件
    rl_command.output_file.close()
