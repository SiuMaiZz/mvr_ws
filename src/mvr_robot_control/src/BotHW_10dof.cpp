//-----------------------------------------------------------------------------
// File: BotHW_10dof.cpp
// Author: Akisora
// Created: 2026-01-22
//-----------------------------------------------------------------------------

#include "mvr_robot_control/BotHW_10dof.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <mvr_robot_control/ObserveData.h>
#include <mvr_robot_control/ActionData.h>
#include <mvr_robot_control/TestData.h>
#include <mvr_robot_control/ObserveData_10dof.h>
#include <mvr_robot_control/ActionData_10dof.h>


static inline double smooth01(double t, double T) {
  if (T <= 0.0) return 1.0;
  if (t <= 0.0) return 0.0;
  if (t >= T)   return 1.0;
  const double u = t / T;
  return 0.5 - 0.5 * std::cos(M_PI * u);
}

static inline double smooth01_dot(double t, double T) {
  if (T <= 0.0) return 0.0;
  if (t <= 0.0) return 0.0;
  if (t >= T)   return 0.0;
  const double u = t / T;
  return (M_PI / (2.0 * T)) * std::sin(M_PI * u);
}

std::vector<int> motor_ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                              12, 13, 14, 15, 16, 17, 18, 19, 20, 21};

bool BotHW::init(ros::NodeHandle& nh) {
    nh.setParam("bot_hardware_interface", "null");

    imu_sub_     = nh.subscribe("/imu/data", 1000, &BotHW::odomCallback, this);
    command_sub_ = nh.subscribe("/control_cmd", 1000, &BotHW::commandCallback, this);
    observe_pub_ = nh.advertise<mvr_robot_control::ObserveData_10dof>("/observe_data", 50);
    // observe_pub_ = nh.advertise<mvr_robot_control::TestData>("/observe_data", 1000);

    left_solver = std::make_unique<MechanismSolver>(l1, d1, h1, h2, MechanismSolver::Side::Left);
    right_solver = std::make_unique<MechanismSolver>(l1, d1, h1, h2, MechanismSolver::Side::Right);

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
            // ROS_INFO_STREAM("Loaded kp and kd for motor " << motor_id << ": " << joint_kps_[motor_id] << ", " << joint_kds_[motor_id]);
        } else {
            ROS_WARN_STREAM("Failed to load kp or kd for motor " << motor_id);
        }
    }

    smoothStartupToDefault(3.0, 50, 50.0);

    std::vector<std::string> joint_names {
        "left_hip_roll_joint",  "left_hip_yaw_joint",   "left_hip_pitch_joint",  "left_knee_joint",  "left_ankle_pitch_joint",
        "right_hip_roll_joint", "right_hip_yaw_joint",  "right_hip_pitch_joint", "right_knee_joint", "right_ankle_pitch_joint"
    };


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

    ROS_INFO_STREAM("BotHW Initial Joints Number: " << joint_names.size());
    return true;
}

bool BotHW::smoothStartupToDefault(double ramp_duration_sec,
                                  int warmup_frames,
                                  double hz) {
    if (hz <= 1e-6) hz = 50.0;
    ros::Rate rate(hz);

    ROS_INFO_STREAM("[init] smoothStartupToDefault begin. warmup_frames="
                    << warmup_frames << ", ramp=" << ramp_duration_sec
                    << "s, hz=" << hz);
  
    ROS_INFO("[init] Stage=BOOTSTRAP: send dummy cmd then get state.");
    for (int motor_id : motor_ids) {
        mvrSendcmd_[motor_id].kp_      = 0.0;
        mvrSendcmd_[motor_id].kd_      = 0.0;
        mvrSendcmd_[motor_id].ff_      = 0.0;
        mvrSendcmd_[motor_id].vel_des_ = 0.0;
        mvrSendcmd_[motor_id].pos_des_ = default_joint_positions_[motor_id];
    }

    EtherCAT_Send_Command_New((YKSMotorData*)mvrSendcmd_);
    EtherCAT_Get_State_New();   
    rate.sleep();

    ROS_INFO_STREAM("[init] Stage=WARMUP: hold current pos with kp/kd=0 for "
                    << warmup_frames << " frames.");
    for (int k = 0; k < warmup_frames && ros::ok(); ++k) {
        for (int motor_id : motor_ids) {
            mvrSendcmd_[motor_id].pos_des_ = motorDate_recv[motor_id].pos_;
            mvrSendcmd_[motor_id].vel_des_ = 0.0;
            mvrSendcmd_[motor_id].kp_      = 0.0;
            mvrSendcmd_[motor_id].kd_      = 0.0;
            mvrSendcmd_[motor_id].ff_      = 0.0;
        }
        EtherCAT_Send_Command_New((YKSMotorData*)mvrSendcmd_);
        EtherCAT_Get_State_New();
        rate.sleep();

        if ((k % 10) == 0) {
            ROS_INFO_STREAM("[init] WARMUP progress: " << k << "/" << warmup_frames);
        }
    }

    ROS_INFO("[init] Stage=CAPTURE: capture q_init and targets.");
    std::array<double, TOTAL_MOTORS> q_init{};
    std::array<double, TOTAL_MOTORS> q_target{};
    std::array<double, TOTAL_MOTORS> kp_target{};
    std::array<double, TOTAL_MOTORS> kd_target{};

    for (int motor_id : motor_ids) {
        q_init[motor_id]    = motorDate_recv[motor_id].pos_;
        q_target[motor_id]  = default_joint_positions_[motor_id];
        kp_target[motor_id] = joint_kps_[motor_id];
        kd_target[motor_id] = joint_kds_[motor_id];
    }

    const double dt = 1.0 / hz;
    const int ramp_steps = std::max(1, (int)std::ceil(ramp_duration_sec * hz));

    ROS_INFO_STREAM("[init] Stage=RAMP: cosine ramp to default for "
                    << ramp_steps << " steps.");

    int last_print_percent = -1;
    for (int n = 0; n < ramp_steps && ros::ok(); ++n) {
        const double t  = n * dt;
        const double s  = smooth01(t, ramp_duration_sec);
        const double sd = smooth01_dot(t, ramp_duration_sec);

        for (int motor_id : motor_ids) {
            const double dq = q_target[motor_id] - q_init[motor_id];

            mvrSendcmd_[motor_id].pos_des_ = q_init[motor_id] + s * dq;
            mvrSendcmd_[motor_id].vel_des_ = sd * dq;
            mvrSendcmd_[motor_id].kp_      = s * kp_target[motor_id];
            mvrSendcmd_[motor_id].kd_      = s * kd_target[motor_id];
            mvrSendcmd_[motor_id].ff_      = 0.0;
        }

        EtherCAT_Send_Command_New((YKSMotorData*)mvrSendcmd_);
        EtherCAT_Get_State_New();     
        rate.sleep();

        int percent = (int)std::floor(100.0 * (double)n / (double)(ramp_steps - 1));
        if (percent != last_print_percent && (percent % 10) == 0) {
            last_print_percent = percent;
            ROS_INFO_STREAM("[init] RAMP progress: " << percent << "% (t=" << t << "s)");
        }
    }

    ROS_INFO("[init] Stage=FINALIZE: lock default pos and full gains.");
    for (int motor_id : motor_ids) {
        mvrSendcmd_[motor_id].pos_des_ = q_target[motor_id];
        mvrSendcmd_[motor_id].vel_des_ = 0.0;
        mvrSendcmd_[motor_id].kp_      = kp_target[motor_id];
        mvrSendcmd_[motor_id].kd_      = kd_target[motor_id];
        mvrSendcmd_[motor_id].ff_      = 0.0;

        jointCommand_[motor_id].pos_des_ = q_target[motor_id];
    }

    EtherCAT_Send_Command_New((YKSMotorData*)mvrSendcmd_);
    EtherCAT_Get_State_New(); 
    rate.sleep();

    ROS_INFO_STREAM("[init] smoothStartupToDefault finished. T=" << ramp_duration_sec
                    << "s, warmup_frames=" << warmup_frames << ", hz=" << hz);
    return true;
}



void BotHW::read(ros::Time time, ros::Duration period) {

    EtherCAT_Get_State_New();

    
    for (int i = 0; i < TOTAL_MOTORS; ++i) {
        jointCommand_[i].pos_ = motorDate_recv[i].pos_;
        jointCommand_[i].vel_ = motorDate_recv[i].vel_;
        jointCommand_[i].tau_ = motorDate_recv[i].tau_;
    }

     sensor_msgs::Imu imu_copy;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_copy = yesenceIMU_;
    }

    // ROS_INFO_STREAM("Test1 !!!!");
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

    mvr_robot_control::ObserveData_10dof observe_msg;
    observe_msg.header.stamp = ros::Time::now();

    observe_msg.imu_angular_vel[0] = imu_copy.angular_velocity.x;
    observe_msg.imu_angular_vel[1] = imu_copy.angular_velocity.y;
    observe_msg.imu_angular_vel[2] = imu_copy.angular_velocity.z;

    observe_msg.commands[0] = 0.2;
    observe_msg.commands[1] = 0.0;
    observe_msg.commands[2] = 0.0;

    observe_msg.quat_float[0] = imu_copy.orientation.x;
    observe_msg.quat_float[1] = imu_copy.orientation.y;
    observe_msg.quat_float[2] = imu_copy.orientation.z;
    observe_msg.quat_float[3] = imu_copy.orientation.w;


    observe_msg.joint_pos[0] = jointCommand_[0].pos_;
    observe_msg.joint_pos[1] = jointCommand_[1].pos_;
    observe_msg.joint_pos[2] = jointCommand_[2].pos_;
    observe_msg.joint_pos[3] = jointCommand_[3].pos_;
    observe_msg.joint_vel[0] = jointCommand_[0].vel_;
    observe_msg.joint_vel[1] = jointCommand_[1].vel_;
    observe_msg.joint_vel[2] = jointCommand_[2].vel_;
    observe_msg.joint_vel[3] = jointCommand_[3].vel_;

    observe_msg.joint_pos[5] = jointCommand_[6].pos_;
    observe_msg.joint_pos[6] = jointCommand_[7].pos_;
    observe_msg.joint_pos[7] = jointCommand_[8].pos_;
    observe_msg.joint_pos[8] = jointCommand_[9].pos_;
    observe_msg.joint_vel[5] = jointCommand_[6].vel_;
    observe_msg.joint_vel[6] = jointCommand_[7].vel_;
    observe_msg.joint_vel[7] = jointCommand_[8].vel_;
    observe_msg.joint_vel[8] = jointCommand_[9].vel_;

    
    if (left_solver && right_solver) {
        {
            double tp = 0.0, tr = 0.0;
            if (left_solver->forwardKinematics(jointCommand_[4].pos_, jointCommand_[5].pos_, tp, tr)) {
                observe_msg.joint_pos[4] = tp;
                // observe_msg.joint_pos[5] = tr;

                double tpdot = 0.0, trdot = 0.0;
                if (left_solver->motorVelToJointVel(jointCommand_[4].pos_, jointCommand_[5].pos_,
                                                tp, tr,
                                                jointCommand_[4].vel_, jointCommand_[5].vel_,
                                                tpdot, trdot)) {
                    observe_msg.joint_vel[4] = tpdot;
                    // observe_msg.joint_vel[5] = trdot;
                } else {
                    ROS_WARN_STREAM("Left ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
                }
            } else {
                ROS_WARN_STREAM("Left ankle FK failed. Keep observe_msg.joint_pos[4],[5] as-is.");
            }
        }

        {
            double tp = 0.0, tr = 0.0;
            if (right_solver->forwardKinematics(jointCommand_[10].pos_, jointCommand_[11].pos_, tp, tr)) {
                observe_msg.joint_pos[9] = tp;
                // observe_msg.joint_pos[10] = tr;

                double tpdot = 0.0, trdot = 0.0;
                if (right_solver->motorVelToJointVel(jointCommand_[10].pos_, jointCommand_[11].pos_,
                                                tp, tr,
                                                jointCommand_[10].vel_, jointCommand_[11].vel_,
                                                tpdot, trdot)) {
                    observe_msg.joint_vel[9] = tpdot;
                    // observe_msg.joint_vel[10] = trdot;
                } else {
                    ROS_WARN_STREAM("Right ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
                }
            } else {
                ROS_WARN_STREAM("Right ankle FK failed. Keep observe_msg.joint_pos[10],[11] as-is.");
            }
        }
    }

    observe_pub_.publish(observe_msg);



}

void BotHW::write(ros::Time time, ros::Duration period) {

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

    EtherCAT_Send_Command_New((YKSMotorData*)mvrSendcmd_);
}

void BotHW::odomCallback(const sensor_msgs::Imu::ConstPtr &odom) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    yesenceIMU_ = *odom;
}

void BotHW::commandCallback(const mvr_robot_control::ActionData_10dof::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    if (emergencyStopFlag_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "In damping brake mode, ignore incoming commands.");
        return;
    }

    if (!(left_solver && right_solver)) {
        ROS_ERROR_STREAM("MechanismSolver not initialized.");
        return;
    }

    jointCommand_[0].pos_des_ = msg->joint_pos[0];
    jointCommand_[1].pos_des_ = msg->joint_pos[1];
    jointCommand_[2].pos_des_ = msg->joint_pos[2];
    jointCommand_[3].pos_des_ = msg->joint_pos[3];

    {
        constexpr double kThetaPLimit = 0.318;

        const double theta_p_raw = msg->joint_pos[4];
        double theta_p = theta_p_raw;
        if (theta_p >  kThetaPLimit) theta_p =  kThetaPLimit;
        if (theta_p < -kThetaPLimit) theta_p = -kThetaPLimit;

        const double theta_r = 0.0;

        if (theta_p != theta_p_raw) {
            ROS_WARN_STREAM_THROTTLE(1.0,
                "[Left ankle] theta_p saturated: raw=" << theta_p_raw << " -> " << theta_p);
        }

        double theta1 = 0.0, theta2 = 0.0;
        if (left_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
            jointCommand_[4].pos_des_ = theta1;
            jointCommand_[5].pos_des_ = theta2;
        } else {
            ROS_WARN_STREAM("Left ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
        }
    }

    jointCommand_[6].pos_des_ = msg->joint_pos[5];
    jointCommand_[7].pos_des_ = msg->joint_pos[6];
    jointCommand_[8].pos_des_ = msg->joint_pos[7];
    jointCommand_[9].pos_des_ = msg->joint_pos[8];

    {
         constexpr double kThetaPLimit = 0.318;

        const double theta_p_raw = msg->joint_pos[9];
        double theta_p = theta_p_raw;
        if (theta_p >  kThetaPLimit) theta_p =  kThetaPLimit;
        if (theta_p < -kThetaPLimit) theta_p = -kThetaPLimit;

        const double theta_r = 0.0;

        if (theta_p != theta_p_raw) {
            ROS_WARN_STREAM_THROTTLE(1.0,
                "[Right ankle] theta_p saturated: raw=" << theta_p_raw << " -> " << theta_p);
        }

        double theta1 = 0.0, theta2 = 0.0;
        if (right_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
            jointCommand_[10].pos_des_ = theta1;  
            jointCommand_[11].pos_des_ = theta2;
        } else {
            ROS_WARN_STREAM("Right ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
        }
    }

}



