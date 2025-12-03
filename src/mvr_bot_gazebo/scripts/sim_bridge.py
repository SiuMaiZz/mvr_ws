#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64
# 注意：包名已根据你的项目名为 mvr_bot_gazebo
from mvr_bot_gazebo.msg import ObserveData, ActionData 

class SimHardwareBridge:
    def __init__(self):
        rospy.init_node('sim_hardware_bridge')

        # === 配置目标关节 ===
        # 根据你提供的 RL 代码中的模型名 'policy_1_right_arm_pitch_high.pt'
        # 我们针对 'right_arm_pitch_higher_joint' 进行单关节控制验证
        self.target_joint_name = 'right_arm_pitch_higher_joint'
        self.target_controller_topic = '/mvr_bot/right_arm_pitch_higher_controller/command'

        # === PD 参数 (关键！) ===
        # RL 输出的是位置 (Position)，但 Gazebo 现在是力控 (Effort)
        # 我们需要一个 PD 控制器来充当伺服电机的角色
        self.kp = 80.0  # 刚度
        self.kd = 3.0   # 阻尼

        # === 内部状态 ===
        self.current_joint_pos = 0.0
        self.current_joint_vel = 0.0
        self.latest_imu = None
        
        # === 1. 订阅 Gazebo 数据 ===
        rospy.Subscriber('/mvr_bot/joint_states', JointState, self.joint_state_cb)
        rospy.Subscriber('/imu/data', Imu, self.imu_cb)

        # === 2. 订阅 RL 指令 ===
        rospy.Subscriber('/control_cmd', ActionData, self.action_cb)

        # === 3. 发布观测数据给 RL ===
        self.obs_pub = rospy.Publisher('/observe_data', ObserveData, queue_size=1)

        # === 4. 发布力矩给 Gazebo ===
        self.gazebo_cmd_pub = rospy.Publisher(self.target_controller_topic, Float64, queue_size=1)

        # === 5. 定时器 (100Hz) ===
        rospy.Timer(rospy.Duration(0.01), self.control_loop)

        rospy.loginfo(f"Sim Bridge Initialized. Controlling: {self.target_joint_name}")

    def joint_state_cb(self, msg):
        """ 从 joint_states 中提取目标关节的位置和速度 """
        try:
            if self.target_joint_name in msg.name:
                idx = msg.name.index(self.target_joint_name)
                self.current_joint_pos = msg.position[idx]
                self.current_joint_vel = msg.velocity[idx]
        except ValueError:
            pass

    def imu_cb(self, msg):
        self.latest_imu = msg

    def action_cb(self, msg):
        """ 接收 RL 的目标位置 -> 计算力矩 -> 发送给 Gazebo """
        # 引用 ActionData.msg 定义: float32[1] joint_pos 
        if not msg.joint_pos:
            return

        target_pos = msg.joint_pos[0]
        
        # PD 控制公式: Torque = Kp * (Target - Current) - Kd * Velocity
        error = target_pos - self.current_joint_pos
        torque = (self.kp * error) - (self.kd * self.current_joint_vel)

        # 安全限幅 (+- 60Nm)
        torque = np.clip(torque, -60.0, 60.0)

        # 发送到 Gazebo
        self.gazebo_cmd_pub.publish(Float64(torque))

    def control_loop(self, event):
        """ 打包 Gazebo 数据为 ObserveData """
        if self.latest_imu is None:
            return

        # 引用 ObserveData.msg 定义 
        obs_msg = ObserveData()
        obs_msg.header.stamp = rospy.Time.now()

        # 1. IMU 角速度
        obs_msg.imu_angular_vel = [
            self.latest_imu.angular_velocity.x,
            self.latest_imu.angular_velocity.y,
            self.latest_imu.angular_velocity.z
        ]
        
        # 2. 四元数
        obs_msg.quat_float = [
            self.latest_imu.orientation.x,
            self.latest_imu.orientation.y,
            self.latest_imu.orientation.z,
            self.latest_imu.orientation.w
        ]

        # 3. 关节数据
        obs_msg.joint_pos = [self.current_joint_pos]
        obs_msg.joint_vel = [self.current_joint_vel]
        
        # 4. 指令回传 (占位)
        obs_msg.commands = [0.0, 0.0, 0.0]

        self.obs_pub.publish(obs_msg)

if __name__ == '__main__':
    try:
        SimHardwareBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass