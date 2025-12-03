#!/usr/bin/env python3

import os
import torch
import torch.nn as nn # type: ignore
import rospy
import numpy as np
from mvr_robot_control.msg import ObserveData, ActionData
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

class ROSNode:
    def __init__(self):
        rospy.init_node('rl_model_command')


# obs_buf = torch.cat((
#             self.command_input,  # 5 = 2D(sin cos) + 3D(vel_x, vel_y, aug_vel_yaw)
#             q,    # 22
#             dq,  # 22
#             self.actions,   # 22
#             self.base_ang_vel * self.obs_scales.ang_vel,  # 3
#             self.base_euler_xyz * self.obs_scales.quat,  # 3
#         ), dim=-1)   
        self.history_buffer = np.zeros((15, 14), dtype=np.float32)
        self.buffer_ptr = 0

        self.last_action = np.zeros(1, dtype=np.float32)

        script_path = os.path.dirname(os.path.realpath(__file__))

        model_relative_path = os.path.join('..', 'model', 'policy_1_right_arm_pitch_high.pt')

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
        
        self.sub = rospy.Subscriber('/observe_data', ObserveData, self.callback)
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

        self.default_pos = 0

    def _get_phase(self):

        # obs[0] = math.sin(2 * math.pi * self.count * self.cfg.dt / 0.64)
        # obs[1] = math.cos(2 * math.pi * self.count * self.cfg.dt / 0.64)

        cycle_time = 0.64
        # elapsed = (rospy.Time.now() - msg.header.stamp).to_sec()
        dt = 0.001
        phase = self.count * dt / cycle_time
        return phase

    def compute_obs(self, msg):


        joint_pos = np.array(msg.joint_pos, dtype=np.float32) * self.obs_scales['dof_pos']
        joint_vel = np.array(msg.joint_vel, dtype=np.float32) * self.obs_scales['dof_vel']
        # euler_angles = np.array(msg.quat_float, dtype=np.float32) * self.obs_scales['quat']
        quat = np.array(msg.quat_float, dtype=np.float32)
        r = R.from_quat(quat)
        euler_angles = r.as_euler('xyz') * self.obs_scales['quat']
        commands = np.array(msg.commands, dtype=np.float32)
        ang_vel = np.array(msg.imu_angular_vel) * self.obs_scales['ang_vel']

        phase = self._get_phase()
        sin_pos = np.sin(2 * np.pi * phase)
        cos_pos = np.cos(2 * np.pi * phase)

        obs = np.concatenate([
            np.array([sin_pos, cos_pos], dtype=np.float32),
            np.array(commands, dtype=np.float32),
            np.array(joint_pos - self.default_pos, dtype=np.float32),
            np.array(joint_vel, dtype=np.float32),
            np.array(self.last_action, dtype=np.float32),
            np.array(ang_vel, dtype=np.float32),
            np.array(euler_angles, dtype=np.float32)
        ])

        self.history_buffer[self.buffer_ptr] = obs
        self.buffer_ptr = (self.buffer_ptr + 1) % 15
        
        obs_buf = self.history_buffer.reshape(1,-1)

        rospy.loginfo(f"Joint Positions (joint_pos): {joint_pos}")
        rospy.loginfo(f"Quaternion (quat): {quat}")
        rospy.loginfo(f"Euler Angles (euler_angles): {euler_angles}")
        rospy.loginfo(f"Angular Velocities (ang_vel): {ang_vel}")

        return obs_buf
    
# clip_actions = self.cfg.normalization.clip_actions
#         self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)
# actions_scaled = actions * self.cfg.control.action_scale
# action_scale = 0.25
# clip_actions = 18.

    def callback(self, msg):
        obs = self.compute_obs(msg)
        obs_tensor = torch.FloatTensor(obs).to(self.device).unsqueeze(0)

        action_scale = 0.25
        clip_actions = 18
        
        with torch.no_grad(): 
            action = self.model(obs_tensor)

        action = torch.clip(action, -clip_actions, clip_actions).to(self.device)
        action_scaled = action * action_scale
        action = action_scaled.cpu().numpy().flatten().astype(np.float32)

        
        
        action_msg = ActionData()
        joint_pos = list(action[:1]) 
        # if len(joint_pos) < 1:
        #     joint_pos.extend([0] * (1 - len(joint_pos)))

        action_msg.joint_pos = joint_pos

        self.pub.publish(action_msg)
        self.last_action = action.flatten()
        self.count += 1

        rospy.loginfo(f"Action - Joint Positions (joint_pos): {joint_pos}")


if __name__ == '__main__':
    node = ROSNode()
    rospy.spin()