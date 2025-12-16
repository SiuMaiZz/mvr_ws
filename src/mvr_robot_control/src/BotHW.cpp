//-----------------------------------------------------------------------------
// File: BotHW.cpp
// Author: Akisora
// Created: 2025-11-25
//-----------------------------------------------------------------------------

#include "mvr_robot_control/BotHW.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <mvr_robot_control/ObserveData.h>
#include <mvr_robot_control/ActionData.h>
#include <mvr_robot_control/TestData.h>

// int id = 15;
// double KP_BASE = 7.0;
// double KD_BASE = 5.0;

// std::vector<int> motor_ids = {13, 14, 15, 16, 18, 19, 20, 21};
// std::vector<int> motor_ids = {18, 19, 20, 21};
std::vector<int> motor_ids = {3, 9};
// std::vector<int> motor_ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

bool BotHW::init(ros::NodeHandle& nh) {
    nh.setParam("bot_hardware_interface", "null");

    imu_sub_     = nh.subscribe("/imu/data", 1000, &BotHW::odomCallback, this);
    command_sub_ = nh.subscribe("/control_cmd", 1000, &BotHW::commandCallback, this);
    // observe_pub_ = nh.advertise<mvr_robot_control::ObserveData>("/observe_data", 50);
    observe_pub_ = nh.advertise<mvr_robot_control::TestData>("/observe_data", 1000);

    int ec_slavecount = EtherCAT_Init(const_cast<char*>("enp4s0"));
    std::cout << "开始EtherCAT初始化" << std::endl;
    if (ec_slavecount <= 0) {
        std::cout << "未找到从站，程序退出" << std::endl;
        return false;
    }

    default_joint_positions_.resize(TOTAL_MOTORS);
    for (int i = 0; i < TOTAL_MOTORS; ++i) {
        std::string param_name = "/default_positions/" + std::to_string(i);
        ros::param::get(param_name, default_joint_positions_[i]);
    }
    ROS_INFO_STREAM("Set default positions correctly!");

    joint_limits_.resize(TOTAL_MOTORS);
    for (int i = 0; i < TOTAL_MOTORS; ++i) {
    joint_limits_interface::JointLimits limits;
    std::string param_name = "/joint_limits/" + std::to_string(i);
    ros::param::get(param_name + "/min_position", limits.min_position);
    ros::param::get(param_name + "/max_position", limits.max_position);
    ros::param::get(param_name + "/max_velocity", limits.max_velocity);
    ros::param::get(param_name + "/max_effort", limits.max_effort);
    joint_limits_[i] = limits;
    }
    ROS_INFO_STREAM("Set joint limits correctly!");

    joint_kds_.resize(TOTAL_MOTORS);
    joint_kps_.resize(TOTAL_MOTORS);
    for (int motor_id = 0; motor_id < TOTAL_MOTORS; ++motor_id) {
        std::string param_name_pds = "/joint_pds/" + std::to_string(motor_id);

        if (ros::param::get(param_name_pds + "/kp", joint_kps_[motor_id]) &&
            ros::param::get(param_name_pds + "/kd", joint_kds_[motor_id])) {
            ROS_INFO_STREAM("Loaded kp and kd for motor " << motor_id << ": " << joint_kps_[motor_id] << ", " << joint_kds_[motor_id]);
        } else {
            ROS_WARN_STREAM("Failed to load kp or kd for motor " << motor_id);
        }
    }


    // for (int i = 0; i < TOTAL_MOTORS; ++i) {
    //     jointCommand_[i].pos_des_ = default_joint_positions_[i];

    //     mvrSendDefaultcmd_[i].pos_des_ = default_joint_positions_[i];
    //     mvrSendDefaultcmd_[i].vel_des_ = 0.0;
    //     mvrSendDefaultcmd_[i].kp_ = 0.0;
    //     mvrSendDefaultcmd_[i].kd_ = 0.0;
    //     if (i == id) {
    //         mvrSendDefaultcmd_[i].kp_ = KP_BASE;
    //         mvrSendDefaultcmd_[i].kd_ = KD_BASE;
    //     }
    //     mvrSendDefaultcmd_[i].ff_ = 0.0;
    // }

    for (int i = 0; i < motor_ids.size(); ++i) {
        int motor_id = motor_ids[i];

        jointCommand_[motor_id].pos_des_ = default_joint_positions_[motor_id];
        
        mvrSendDefaultcmd_[motor_id].pos_des_ = default_joint_positions_[motor_id];
        mvrSendDefaultcmd_[motor_id].vel_des_ = 0.0;
        mvrSendDefaultcmd_[motor_id].kp_ = joint_kps_[motor_id];
        mvrSendDefaultcmd_[motor_id].kd_ = joint_kds_[motor_id];
        mvrSendDefaultcmd_[motor_id].ff_ = 0.0; 
    }

    ROS_INFO("Default joint positions initialized statically");

    EtherCAT_Send_Command_New((YKSMotorData*)mvrSendDefaultcmd_);
    EtherCAT_Get_State_New();

    std::vector<std::string> joint_names {
        "left_hip_pitch_joint",  "left_hip_roll_joint",   "left_hip_yaw_joint",  "left_knee_joint",  "left_ankle_pitch_joint",  "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint",  "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_joint", "left_arm_pitch_higher_joint",  "left_arm_roll_joint",  "left_arm_yaw_joint",  "left_arm_pitch_lower_joint",
        "head_joint",  "right_arm_pitch_higher_joint", "right_arm_roll_joint", "right_arm_yaw_joint", "right_arm_pitch_lower_joint"
    };

    std::vector<int> motor_ids(joint_names.size());
    std::iota(motor_ids.begin(), motor_ids.end(), 0);

    if (joint_names.size() > TOTAL_MOTORS) {
        ROS_ERROR_STREAM("joint_names 数量 (" << joint_names.size()
                         << ") 大于 TOTAL_MOTORS (" << TOTAL_MOTORS << ")");
        return false;
    }

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];
        int motor_id = motor_ids[i]; 

        hardware_interface::JointStateHandle state_handle(
            joint_name,
            &jointCommand_[motor_id].pos_,
            &jointCommand_[motor_id].vel_,
            &jointCommand_[motor_id].tau_);
        joint_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(
            joint_state_interface_.getHandle(joint_name),
            &jointCommand_[motor_id].pos_des_);
        joint_position_interface_.registerHandle(pos_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&joint_position_interface_);

    ROS_INFO_STREAM("BotHW 初始化完成，共注册关节数： " << joint_names.size());
    return true;
}

void BotHW::read(ros::Time time, ros::Duration period) {

    EtherCAT_Get_State_New();

    mvr_robot_control::TestData test_msg;
    test_msg.header.stamp = ros::Time::now();

    // jointCommand_[id].pos_ = motorDate_recv[id].pos_;
    // jointCommand_[id].vel_ = motorDate_recv[id].vel_;
    // jointCommand_[id].tau_ = motorDate_recv[id].tau_;

    // test_msg.joint_pos[0] = jointCommand_[id].pos_;
    // test_msg.joint_vel[0] = jointCommand_[id].vel_;

    for (int motor_id : motor_ids) {
        jointCommand_[motor_id].pos_ = motorDate_recv[motor_id].pos_;
        jointCommand_[motor_id].vel_ = motorDate_recv[motor_id].vel_;
        jointCommand_[motor_id].tau_ = motorDate_recv[motor_id].tau_;
    }
    
    int index = 0;
    for (int motor_id : motor_ids) {
        test_msg.joint_pos[index] = jointCommand_[motor_id].pos_;
        test_msg.joint_vel[index] = jointCommand_[motor_id].vel_;
        index++;
    }

    // int out = 0;
    // for (int i = 0; i < 5; ++i) {
    //     int motor_id = motor_ids[i];
    //     test_msg.joint_pos[out] = jointCommand_[motor_id].pos_;
    //     test_msg.joint_vel[out] = jointCommand_[motor_id].vel_;
    //     out++;
    // }
    // for (int i = 6; i <= 10; ++i) {
    //     int motor_id = motor_ids[i];
    //     test_msg.joint_pos[out] = jointCommand_[motor_id].pos_;
    //     test_msg.joint_vel[out] = jointCommand_[motor_id].vel_;
    //     out++;
    // }

    observe_pub_.publish(test_msg);

    sensor_msgs::Imu imu_copy;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_copy = yesenceIMU_;
    }

    imuData_.ori[0] = imu_copy.orientation.x;
    imuData_.ori[1] = imu_copy.orientation.y;
    imuData_.ori[2] = imu_copy.orientation.z;
    imuData_.ori[3] = imu_copy.orientation.w;
    imuData_.angular_vel[0] = imu_copy.angular_velocity.x;
    imuData_.angular_vel[1] = imu_copy.angular_velocity.y;
    imuData_.angular_vel[2] = imu_copy.angular_velocity.z;
    imuData_.linear_acc[0]  = imu_copy.linear_acceleration.x;
    imuData_.linear_acc[1]  = imu_copy.linear_acceleration.y;
    imuData_.linear_acc[2]  = imu_copy.linear_acceleration.z;

    // int print = 0;
    // for (int i = 0; i < 5; ++i) {
    //     ROS_INFO_STREAM("pub idx " << print << " from motor_id " << motor_ids[i]
    //                     << " pos=" << test_msg.joint_pos[print]);
    //     print++;
    // }
    // for (int i = 6; i <= 10; ++i) {
    //     ROS_INFO_STREAM("pub idx " << print << " from motor_id " << motor_ids[i]
    //                     << " pos=" << test_msg.joint_pos[print]);
    //     print++;
    // }

    for (int i = 0; i < motor_ids.size(); ++i) {
        ROS_INFO_STREAM("Joint " << motor_ids[i] << ": " << test_msg.joint_pos[i]);
    }

    // ROS_INFO_STREAM("Torque from Current: ");
    // for (int i = 0; i < motor_ids.size(); ++i) {
    //     ROS_INFO_STREAM("Joint " << motor_ids[i] << ": " << std::abs(motorDate_recv[motor_ids[i]].current_));
    // }




    // ROS_INFO_STREAM("Joint positions: ");
    // ROS_INFO_STREAM(" " << test_msg.joint_pos[0]);




    ROS_INFO_STREAM("Torque from Current 3: ");
    ROS_INFO_STREAM(" "<< std::abs(motorDate_recv[3].current_));

    double pos_des_3   = jointCommand_[3].pos_des_;
    double pos_now_3   = jointCommand_[3].pos_;
    double vel_now_3   = jointCommand_[3].vel_;

    double pos_err_3 = pos_des_3 - pos_now_3;
    double torque_pd_3   = joint_kps_[3] * pos_err_3 - joint_kds_[3] * vel_now_3;

    ROS_INFO_STREAM("Torque from PD formula 9: ");
    ROS_INFO_STREAM(" " << torque_pd_3);

    ROS_INFO_STREAM("Torque from Current 9: ");
    ROS_INFO_STREAM(" "<< std::abs(motorDate_recv[9].current_));

    double pos_des   = jointCommand_[9].pos_des_;
    double pos_now   = jointCommand_[9].pos_;
    double vel_now   = jointCommand_[9].vel_;

    double pos_err = pos_des - pos_now;
    double torque_pd   = joint_kps_[9] * pos_err - joint_kds_[9] * vel_now;

    ROS_INFO_STREAM("Torque from PD formula 9: ");
    ROS_INFO_STREAM(" " << torque_pd);


    // mvr_robot_control::ObserveData observe_msg;
    // observe_msg.header.stamp = ros::Time::now();

    // observe_msg.imu_angular_vel[0] = imu_copy.angular_velocity.x;
    // observe_msg.imu_angular_vel[1] = imu_copy.angular_velocity.y;
    // observe_msg.imu_angular_vel[2] = imu_copy.angular_velocity.z;

    // observe_msg.joint_pos[0] = jointCommand_[id].pos_;
    // observe_msg.joint_vel[0] = jointCommand_[id].vel_;

    // observe_msg.commands[0] = 0.0;
    // observe_msg.commands[1] = 0.0;
    // observe_msg.commands[2] = 0.0;

    // observe_msg.quat_float[0] = imu_copy.orientation.x;
    // observe_msg.quat_float[1] = imu_copy.orientation.y;
    // observe_msg.quat_float[2] = imu_copy.orientation.z;
    // observe_msg.quat_float[3] = imu_copy.orientation.w;

    // observe_pub_.publish(observe_msg);

    // ROS_INFO_STREAM("ObserveData: ");
    // ROS_INFO_STREAM("Joint positions: ");

    // ROS_INFO_STREAM("  " << observe_msg.joint_pos[0]);

    // ROS_INFO_STREAM("IMU angular velocities: ");
    // ROS_INFO_STREAM("  " << observe_msg.imu_angular_vel[0] << ", " 
    //                      << observe_msg.imu_angular_vel[1] << ", " 
    //                      << observe_msg.imu_angular_vel[2]);
    // ROS_INFO_STREAM("Quaternion: ");
    // ROS_INFO_STREAM("  " << observe_msg.quat_float[0] << ", "
    //                      << observe_msg.quat_float[1] << ", "
    //                      << observe_msg.quat_float[2] << ", "
    //                      << observe_msg.quat_float[3]);
}

void BotHW::write(ros::Time time, ros::Duration period) {
    // for (int i = 0; i < TOTAL_MOTORS; ++i) {
    //     // double desired_pos = 0.0;
    //     double desired_pos = jointCommand_[i].pos_des_;

    //     desired_pos = std::max(joint_limits_[i].min_position, desired_pos);
    //     desired_pos = std::min(joint_limits_[i].max_position, desired_pos);

    //     mvrSendcmd_[i].pos_des_ = desired_pos;
    //     mvrSendcmd_[i].vel_des_ = 0.0; 
    //     mvrSendcmd_[i].kp_      = 0.0;
    //     mvrSendcmd_[i].kd_      = 0.0;
    //     if(i == id){
    //         mvrSendcmd_[i].kp_      = KP_BASE;
    //         mvrSendcmd_[i].kd_      = KD_BASE;
    //     }
    //     mvrSendcmd_[i].ff_      = 0.0;
    // }

    for (int motor_id = 0; motor_id < TOTAL_MOTORS; ++motor_id) {
        double desired_pos = jointCommand_[motor_id].pos_des_;
        
        desired_pos = std::max(joint_limits_[motor_id].min_position, desired_pos);
        desired_pos = std::min(joint_limits_[motor_id].max_position, desired_pos);

        mvrSendcmd_[motor_id].pos_des_ = desired_pos;
        mvrSendcmd_[motor_id].vel_des_ = 0.0; 
        mvrSendcmd_[motor_id].kp_ = joint_kps_[motor_id];     
        mvrSendcmd_[motor_id].kd_ = joint_kds_[motor_id];
        mvrSendcmd_[motor_id].ff_ = 0.0;  
    }

    EtherCAT_Send_Command_New(mvrSendcmd_);
}

void BotHW::odomCallback(const sensor_msgs::Imu::ConstPtr &odom) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    yesenceIMU_ = *odom;
}

void BotHW::commandCallback(const mvr_robot_control::ActionData::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    // SINGLE MOTOR

    // ROS_INFO_STREAM("Received ActionData: ");
    // ROS_INFO_STREAM("  Joint "<< id << ": " << msg->joint_pos[0]);

    // if (!msg->joint_pos.empty()) {
    //     jointCommand_[id].pos_des_ = msg->joint_pos[0];
    //     // jointCommand_[id].pos_des_ = 0 - msg->joint_pos[0];
    // }

    // if (msg->joint_pos.size() != motor_ids.size()) {
    //     ROS_WARN_STREAM("Expected " << motor_ids.size() << " joint positions, but received " << msg->joint_pos.size());
    // }

    // VECTOR
    for (int i = 0; i < motor_ids.size(); ++i) {
        int motor_id = motor_ids[i];

        ROS_INFO_STREAM("  Joint " << motor_id << ": " << msg->joint_pos[i]);
        
        jointCommand_[motor_id].pos_des_ = msg->joint_pos[i];
    }


    // 5Dof
    // double target12[12];

    // for (int i = 0; i < 5; ++i) {
    //     target12[i] = msg->joint_pos[i];
    // }
    // target12[5] = -msg->joint_pos[4];

    // for (int i = 0; i < 5; ++i) {
    //     target12[6 + i] = msg->joint_pos[5 + i];
    // }
    // target12[11] = -msg->joint_pos[9];

    // for (int i = 0; i < 12; ++i) {
    //     int motor_id = motor_ids[i];
    //     jointCommand_[motor_id].pos_des_ = target12[i];
    //     ROS_INFO_STREAM("motor_id " << motor_id << " <= target[" << i << "] = " << target12[i]);
    // }
}



