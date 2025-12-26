#!/usr/bin/env python3

import os
import torch
import torch.nn as nn # type: ignore
import rospy
import numpy as np
import csv
from mvr_robot_control.msg import ObserveData, ActionData, TestData
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

class ROSNode:
    def __init__(self):
        rospy.init_node('rl_model_command')

        self.motor_nums = 22

        self.csv_file = open('/home/robot007/mvr_ws/src/mvr_robot_control/data/record_22dof_v7.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['step', 'phase', 'obs', 'action_scaled', 'smoothed_joint_pos'])  # 表头

        

# obs_buf = torch.cat((
#             self.command_input,  # 5 = 2D(sin cos) + 3D(vel_x, vel_y, aug_vel_yaw)
#             q,    # 22
#             dq,  # 22
#             self.actions,   # 22
#             self.base_ang_vel * self.obs_scales.ang_vel,  # 3
#             self.base_euler_xyz * self.obs_scales.quat,  # 3
#         ), dim=-1)   
        self.history_buffer = np.zeros((1, self.motor_nums * 3 + 2), dtype=np.float32)
        # print(self.history_buffer.shape, 'A')
        self.buffer_ptr = 0

        self.last_action = np.zeros(self.motor_nums, dtype=np.float32)

        self.obs_raw = None
        self.phase_raw = None
        self.action_clipped = None
        self.action = None

        script_path = os.path.dirname(os.path.realpath(__file__))

        model_relative_path = os.path.join('..', 'model', 'policy_1_22dof_v7.pt')

        model_path = os.path.abspath(os.path.join(script_path, model_relative_path))

        torch.set_num_threads(4)
        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True     
        
        try:
            self.model = torch.jit.load(model_path)
            rospy.loginfo(f"Loading model from: {model_path}")
            rospy.loginfo("Model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Model load failed: {str(e)}")
            exit(1)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        rospy.loginfo(f"Using device: {self.device}")
        
        self.sub = rospy.Subscriber('/observe_data', TestData, self.callback)
        self.pub = rospy.Publisher('/control_cmd', ActionData, queue_size=1)

        self.count = 0



        # class normalization:
        # class obs_scales:
        #     lin_vel = 2.
        #     ang_vel = 1.
        #     dof_pos = 1.
        #     dof_vel = 0.05
        #     quat = 1.
        #     height_measurements = 5.0
        # clip_observations = 18.
        # clip_actions = 18.

        self.obs_scales = {
            'dof_pos': 1,
            'dof_vel': 0.05,
            'ang_vel': 1,
            'lin_vel': 2,
            'quat': 1,
        }

        self.default_pos = np.array([
            -0.25, -0.02, -0.01, -0.5,  0.3, -0.01,
            0.25,   0.02, -0.01,  0.5, -0.3,  0.01,
            0.0, 0.0, 
            0.0, 0.0, 0.0, -1.0,
            0.0, 0.0, 0.0,  1.0
        ], dtype=np.float32)

        self.smoothed_joint_pos = np.array(self.default_pos, dtype=np.float32)  # 初始平滑值设置为默认关节位置
        self.smooth_alpha = 0.3  # 平滑系数，可以根据需求调整

        self.last_three_joint_pos = np.zeros((3, self.motor_nums), dtype=np.float32)
        self.smooth_weights = np.array([0.3, 0.4, 0.3])  

        self.last_five_joint_pos = np.zeros((5, self.motor_nums), dtype=np.float32)
        self.smooth_weights = np.array([0.2, 0.2, 0.2, 0.2, 0.2]) 

        self.temp_joint_pos = np.zeros(self.motor_nums, dtype=np.float32)

    def _get_phase(self):

        # obs[0] = math.sin(2 * math.pi * self.count * self.cfg.dt / 0.64)
        # obs[1] = math.cos(2 * math.pi * self.count * self.cfg.dt / 0.64)

        cycle_time = 1.28
        # elapsed = (rospy.Time.now() - msg.header.stamp).to_sec()
        dt = 0.01
        phase = self.count * dt / cycle_time
        return phase

    def compute_obs(self, msg):


        joint_pos = np.array(msg.joint_pos, dtype=np.float32) * self.obs_scales['dof_pos']
        joint_vel = np.array(msg.joint_vel, dtype=np.float32) * self.obs_scales['dof_vel']
        # euler_angles = np.array(msg.quat_float, dtype=np.float32) * self.obs_scales['quat']
        
        # quat = np.array(msg.quat_float, dtype=np.float32)
        # r = R.from_quat(quat)
        # euler_angles = r.as_euler('xyz') * self.obs_scales['quat']
        # commands = np.array(msg.commands, dtype=np.float32)
        # ang_vel = np.array(msg.imu_angular_vel) * self.obs_scales['ang_vel']

        phase = self._get_phase()
        self.phase_raw = phase
        sin_pos = np.sin(2 * np.pi * phase)
        cos_pos = np.cos(2 * np.pi * phase)

        obs = np.concatenate([
            np.array([sin_pos, cos_pos], dtype=np.float32),
            # np.array(commands, dtype=np.float32),
            np.array(joint_pos - self.default_pos, dtype=np.float32),
            np.array(joint_vel, dtype=np.float32),
            np.array(self.last_action, dtype=np.float32),
            # np.array(ang_vel, dtype=np.float32),
            # np.array(euler_angles, dtype=np.float32)
        ])
        # print(obs.shape)
        # print(joint_pos.shape)
        # print(joint_vel.shape)
        

        self.obs_raw = obs.flatten()
        # print(obs.shape)
        self.history_buffer[self.buffer_ptr] = obs
        self.buffer_ptr = (self.buffer_ptr + 1) % 1
        
        obs_buf = self.history_buffer.reshape(1,-1)

        rospy.loginfo(f"Joint Positions (joint_pos): {joint_pos}")
        # rospy.loginfo(f"Quaternion (quat): {quat}")
        # rospy.loginfo(f"Euler Angles (euler_angles): {euler_angles}")
        # rospy.loginfo(f"Angular Velocities (ang_vel): {ang_vel}")

        return obs_buf
    
# clip_actions = self.cfg.normalization.clip_actions
#         self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)
# actions_scaled = actions * self.cfg.control.action_scale
# action_scale = 0.25
# clip_actions = 18.

    def callback(self, msg):
        # stop_step = 400

        # if self.count >= stop_step:
        #     rospy.loginfo("Stopping publishing!")
        #     return
    
        decimation = 5

        if self.count % decimation != 0:
            obs = self.compute_obs(msg)
            obs_tensor = torch.FloatTensor(obs).to(self.device).unsqueeze(0)
            action_scale = 0.25
            clip_actions = 18

            with torch.no_grad():
                action = self.model(obs_tensor)

            self.last_action = action.cpu().numpy().flatten()

            action = torch.clip(action, -clip_actions, clip_actions).to(self.device)
            self.action_clipped = action.cpu().numpy().flatten().astype(np.float32)

            action_scaled = action * action_scale
            action = action_scaled.cpu().numpy().flatten().astype(np.float32)
            self.action = action

            joint_pos_np = self.default_pos[:self.motor_nums] + action[:self.motor_nums]

            # 保存当前的 joint_pos
            self.last_five_joint_pos[4] = self.last_five_joint_pos[3]
            self.last_five_joint_pos[3] = self.last_five_joint_pos[2]
            self.last_five_joint_pos[2] = self.last_five_joint_pos[1]
            self.last_five_joint_pos[1] = self.last_five_joint_pos[0]
            self.last_five_joint_pos[0] = joint_pos_np

            # 对五次的 joint_pos 进行加权平均
            smoothed_joint_pos_np = np.average(self.last_five_joint_pos, axis=0, weights=self.smooth_weights)
            self.smoothed_joint_pos = smoothed_joint_pos_np  # 更新平滑值

            self.temp_joint_pos = smoothed_joint_pos_np  # 保存当前的平滑值用于发送

        # 每五步发送一次消息
        if self.count % decimation == 0:
            joint_pos = self.temp_joint_pos.astype(np.float32).tolist()

            self.action_msg = ActionData()
            self.action_msg.joint_pos = joint_pos

            # 写入 CSV
            self.csv_writer.writerow([self.count, self.phase_raw, self.obs_raw, self.action, joint_pos])

            self.pub.publish(self.action_msg)

            rospy.loginfo(f"Action  Pos     (joint_pos): {self.action_msg.joint_pos}")

        self.count += 1

        
        # if self.count % decimation == 0:
        #     obs = self.compute_obs(msg)
        #     obs_tensor = torch.FloatTensor(obs).to(self.device).unsqueeze(0)
        #     # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!", obs_tensor.shape)
        #     action_scale = 0.25
        #     clip_actions = 18

        #     with torch.no_grad(): 
        #         action = self.model(obs_tensor)

        #     self.last_action = action.cpu().numpy().flatten()

        #     action = torch.clip(action, -clip_actions, clip_actions).to(self.device)
        #     self.action_clipped = action.cpu().numpy().flatten().astype(np.float32)


        #     action_scaled = action * action_scale
        #     action = action_scaled.cpu().numpy().flatten().astype(np.float32)
        #     self.action = action

        #     joint_pos_np = self.default_pos[:self.motor_nums] + action[:self.motor_nums]
        #     # joint_pos = joint_pos_np.astype(np.float32).tolist()

        #     # smoothed_joint_pos_np = self.smooth_alpha * joint_pos_np + (1 - self.smooth_alpha) * self.smoothed_joint_pos
        #     # self.smoothed_joint_pos = smoothed_joint_pos_np  # 更新平滑值
        #     # joint_pos = smoothed_joint_pos_np.astype(np.float32).tolist()

        #     self.last_three_joint_pos[2] = self.last_three_joint_pos[1]
        #     self.last_three_joint_pos[1] = self.last_three_joint_pos[0]
        #     self.last_three_joint_pos[0] = joint_pos_np

        #     smoothed_joint_pos_np = np.average(self.last_three_joint_pos, axis=0, weights=self.smooth_weights)
        #     self.smoothed_joint_pos = smoothed_joint_pos_np
        #     joint_pos = smoothed_joint_pos_np.astype(np.float32).tolist()


        #     # lo, hi = -0.1, 0.1
        #     # start_idx, end_idx = 12, 21
        #     # if len(joint_pos) > start_idx:
        #     #     end = min(end_idx, len(joint_pos) - 1)
        #     #     for i in range(start_idx, end + 1):
        #     #         if lo <= joint_pos[i] <= hi:
        #     #             joint_pos[i] = 0.0
        #     # if len(joint_pos) < 1:
        #     #     joint_pos.extend([0] * (1 - len(joint_pos)))

        #     self.action_msg = ActionData()
        #     self.action_msg.joint_pos = joint_pos

        #     self.csv_writer.writerow([self.count, self.phase_raw, self.obs_raw, self.action, joint_pos])

        # self.pub.publish(self.action_msg)
        
        # rospy.loginfo(f"Action  Pos     (joint_pos): {self.action_msg.joint_pos}")
            

        self.count += 1

        


if __name__ == '__main__':
    node = ROSNode()
    rospy.spin()