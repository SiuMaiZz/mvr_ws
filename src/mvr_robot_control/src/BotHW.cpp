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
// ankle_pitch_limit [-0.3176, 0.3176]
// ankle_roll_limit  [-0.3728, 0.3728]
// std::vector<int> motor_ids = {13, 14, 15, 16, 18, 19, 20, 21};
// std::vector<int> motor_ids = {18, 19, 20, 21};
// std::vector<int> motor_ids = {4, 10};
// std::vector<int> motor_ids = {0, 6};
// std::vector<int> motor_ids = {3, 9};
// std::vector<int> motor_ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
// std::vector<int> motor_ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
//                               12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
std::vector<int> motor_ids = {4, 5, 10, 11};

bool BotHW::init(ros::NodeHandle& nh) {
    nh.setParam("bot_hardware_interface", "null");

    imu_sub_     = nh.subscribe("/imu/data", 1000, &BotHW::odomCallback, this);
    command_sub_ = nh.subscribe("/control_cmd", 1000, &BotHW::commandCallback, this);
    // observe_pub_ = nh.advertise<mvr_robot_control::ObserveData>("/observe_data", 50);
    observe_pub_ = nh.advertise<mvr_robot_control::TestData>("/observe_data", 1000);

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
            ROS_INFO_STREAM("Loaded kp and kd for motor " << motor_id << ": " << joint_kps_[motor_id] << ", " << joint_kds_[motor_id]);
        } else {
            ROS_WARN_STREAM("Failed to load kp or kd for motor " << motor_id);
        }
    }

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

    // std::vector<int> motor_ids(joint_names.size());
    // std::iota(motor_ids.begin(), motor_ids.end(), 0);

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

void BotHW::read(ros::Time time, ros::Duration period) {

    EtherCAT_Get_State_New();

    mvr_robot_control::TestData test_msg;
    test_msg.header.stamp = ros::Time::now();

    // test_msg.joint_pos[0] = jointCommand_[id].pos_;
    // test_msg.joint_vel[0] = jointCommand_[id].vel_;


    for (int i = 0; i < TOTAL_MOTORS; ++i) {
        jointCommand_[i].pos_ = motorDate_recv[i].pos_;
        jointCommand_[i].vel_ = motorDate_recv[i].vel_;
        jointCommand_[i].tau_ = motorDate_recv[i].tau_;
    }

    
    // 5Dof
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


    // Whole Body 22dof
    // test_msg.joint_pos[0] = jointCommand_[0].pos_;
    // test_msg.joint_pos[1] = jointCommand_[1].pos_;
    // test_msg.joint_pos[2] = jointCommand_[2].pos_;
    // test_msg.joint_pos[3] = jointCommand_[3].pos_;
    // test_msg.joint_vel[0] = jointCommand_[0].vel_;
    // test_msg.joint_vel[1] = jointCommand_[1].vel_;
    // test_msg.joint_vel[2] = jointCommand_[2].vel_;
    // test_msg.joint_vel[3] = jointCommand_[3].vel_;

    // test_msg.joint_pos[6] = jointCommand_[6].pos_;
    // test_msg.joint_pos[7] = jointCommand_[7].pos_;
    // test_msg.joint_pos[8] = jointCommand_[8].pos_;
    // test_msg.joint_pos[9] = jointCommand_[9].pos_;
    // test_msg.joint_vel[6] = jointCommand_[6].vel_;
    // test_msg.joint_vel[7] = jointCommand_[7].vel_;
    // test_msg.joint_vel[8] = jointCommand_[8].vel_;
    // test_msg.joint_vel[9] = jointCommand_[9].vel_;

    // for (int msg_idx = 12; msg_idx <= 21; ++msg_idx) {
    //     const int motor_id = msg_idx; // 10->12 ... 19->21
    //     test_msg.joint_pos[msg_idx] = jointCommand_[motor_id].pos_;
    //     test_msg.joint_vel[msg_idx] = jointCommand_[motor_id].vel_;
    // }

    // if (left_solver && right_solver) {
    //     {
    //         double tp = 0.0, tr = 0.0;
    //         if (left_solver->forwardKinematics(jointCommand_[4].pos_, jointCommand_[5].pos_, tp, tr)) {
    //             test_msg.joint_pos[4] = tp;
    //             test_msg.joint_pos[5] = tr;

    //             double tpdot = 0.0, trdot = 0.0;
    //             if (left_solver->motorVelToJointVel(jointCommand_[4].pos_, jointCommand_[5].pos_,
    //                                             tp, tr,
    //                                             jointCommand_[4].vel_, jointCommand_[5].vel_,
    //                                             tpdot, trdot)) {
    //                 test_msg.joint_vel[4] = tpdot;
    //                 test_msg.joint_vel[5] = trdot;
    //             } else {
    //                 ROS_WARN_STREAM("Left ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
    //             }
    //         } else {
    //             ROS_WARN_STREAM("Left ankle FK failed. Keep test_msg.joint_pos[4],[5] as-is.");
    //         }
    //     }

    //     {
    //         double tp = 0.0, tr = 0.0;
    //         if (right_solver->forwardKinematics(jointCommand_[10].pos_, jointCommand_[11].pos_, tp, tr)) {
    //             test_msg.joint_pos[10] = tp;
    //             test_msg.joint_pos[11] = tr;

    //             double tpdot = 0.0, trdot = 0.0;
    //             if (right_solver->motorVelToJointVel(jointCommand_[10].pos_, jointCommand_[11].pos_,
    //                                             tp, tr,
    //                                             jointCommand_[10].vel_, jointCommand_[11].vel_,
    //                                             tpdot, trdot)) {
    //                 test_msg.joint_vel[10] = tpdot;
    //                 test_msg.joint_vel[11] = trdot;
    //             } else {
    //                 ROS_WARN_STREAM("Right ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
    //             }
    //         } else {
    //             ROS_WARN_STREAM("Right ankle FK failed. Keep test_msg.joint_pos[10],[11] as-is.");
    //         }
    //     }
    // }

    // Whole Body 20dof
    // test_msg.joint_pos[0] = jointCommand_[0].pos_;
    // test_msg.joint_pos[1] = jointCommand_[1].pos_;
    // test_msg.joint_pos[2] = jointCommand_[2].pos_;
    // test_msg.joint_pos[3] = jointCommand_[3].pos_;
    // test_msg.joint_vel[0] = jointCommand_[0].vel_;
    // test_msg.joint_vel[1] = jointCommand_[1].vel_;
    // test_msg.joint_vel[2] = jointCommand_[2].vel_;
    // test_msg.joint_vel[3] = jointCommand_[3].vel_;

    // test_msg.joint_pos[5] = jointCommand_[6].pos_;
    // test_msg.joint_pos[6] = jointCommand_[7].pos_;
    // test_msg.joint_pos[7] = jointCommand_[8].pos_;
    // test_msg.joint_pos[8] = jointCommand_[9].pos_;
    // test_msg.joint_vel[5] = jointCommand_[6].vel_;
    // test_msg.joint_vel[6] = jointCommand_[7].vel_;
    // test_msg.joint_vel[7] = jointCommand_[8].vel_;
    // test_msg.joint_vel[8] = jointCommand_[9].vel_;

    // for (int msg_idx = 10; msg_idx <= 19; ++msg_idx) {
    //     const int motor_id = msg_idx + 2; // 10->12 ... 19->21
    //     test_msg.joint_pos[msg_idx] = jointCommand_[motor_id].pos_;
    //     test_msg.joint_vel[msg_idx] = jointCommand_[motor_id].vel_;
    // }

    // if (left_solver && right_solver) {
    //     {
    //         double tp = 0.0, tr = 0.0;
    //         if (left_solver->forwardKinematics(jointCommand_[4].pos_, jointCommand_[5].pos_, tp, tr)) {
    //             test_msg.joint_pos[4] = tp;
    //             // test_msg.joint_pos[5] = tr;

    //             double tpdot = 0.0, trdot = 0.0;
    //             if (left_solver->motorVelToJointVel(jointCommand_[4].pos_, jointCommand_[5].pos_,
    //                                             tp, tr,
    //                                             jointCommand_[4].vel_, jointCommand_[5].vel_,
    //                                             tpdot, trdot)) {
    //                 test_msg.joint_vel[4] = tpdot;
    //                 // test_msg.joint_vel[5] = trdot;
    //             } else {
    //                 ROS_WARN_STREAM("Left ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
    //             }
    //         } else {
    //             ROS_WARN_STREAM("Left ankle FK failed. Keep test_msg.joint_pos[4],[5] as-is.");
    //         }
    //     }

    //     {
    //         double tp = 0.0, tr = 0.0;
    //         if (right_solver->forwardKinematics(jointCommand_[10].pos_, jointCommand_[11].pos_, tp, tr)) {
    //             test_msg.joint_pos[9] = tp;
    //             // test_msg.joint_pos[10] = tr;

    //             double tpdot = 0.0, trdot = 0.0;
    //             if (right_solver->motorVelToJointVel(jointCommand_[10].pos_, jointCommand_[11].pos_,
    //                                             tp, tr,
    //                                             jointCommand_[10].vel_, jointCommand_[11].vel_,
    //                                             tpdot, trdot)) {
    //                 test_msg.joint_vel[9] = tpdot;
    //                 // test_msg.joint_vel[10] = trdot;
    //             } else {
    //                 ROS_WARN_STREAM("Right ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
    //             }
    //         } else {
    //             ROS_WARN_STREAM("Right ankle FK failed. Keep test_msg.joint_pos[10],[11] as-is.");
    //         }
    //     }
    // }

    // ankle
    if (left_solver && right_solver) {
        {
            double tp = 0.0, tr = 0.0;
            if (left_solver->forwardKinematics(jointCommand_[4].pos_, jointCommand_[5].pos_, tp, tr)) {
                test_msg.joint_pos[0] = tp;
                test_msg.joint_pos[1] = tr;

                double tpdot = 0.0, trdot = 0.0;
                if (left_solver->motorVelToJointVel(jointCommand_[4].pos_, jointCommand_[5].pos_,
                                                tp, tr,
                                                jointCommand_[4].vel_, jointCommand_[5].vel_,
                                                tpdot, trdot)) {
                    test_msg.joint_vel[0] = tpdot;
                    test_msg.joint_vel[1] = trdot;
                } else {
                    ROS_WARN_STREAM("Left ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
                }
            } else {
                ROS_WARN_STREAM("Left ankle FK failed. Keep test_msg.joint_pos[4],[5] as-is.");
            }
        }

        {
            double tp = 0.0, tr = 0.0;
            if (right_solver->forwardKinematics(jointCommand_[10].pos_, jointCommand_[11].pos_, tp, tr)) {
                test_msg.joint_pos[2] = tp;
                test_msg.joint_pos[3] = tr;

                double tpdot = 0.0, trdot = 0.0;
                if (right_solver->motorVelToJointVel(jointCommand_[10].pos_, jointCommand_[11].pos_,
                                                tp, tr,
                                                jointCommand_[10].vel_, jointCommand_[11].vel_,
                                                tpdot, trdot)) {
                    test_msg.joint_vel[2] = tpdot;
                    test_msg.joint_vel[3] = trdot;
                } else {
                    ROS_WARN_STREAM("Right ankle velocity mapping failed. Keep joint_vel[4],[5] as motor vel.");
                }
            } else {
                ROS_WARN_STREAM("Right ankle FK failed. Keep test_msg.joint_pos[10],[11] as-is.");
            }
        }
    }
    observe_pub_.publish(test_msg);

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


    // SINGLE MOTOR
    // ROS_INFO_STREAM("Joint positions: ");
    // ROS_INFO_STREAM(" " << test_msg.joint_pos[0]);


    // VECTOR
    // for (int i = 0; i < motor_ids.size(); ++i) {
    //     ROS_INFO_STREAM("Joint " << motor_ids[i] << ": " << test_msg.joint_pos[i]);
    // }

    // ROS_INFO_STREAM("Torque from Current: ");
    // for (int i = 0; i < motor_ids.size(); ++i) {
    //     ROS_INFO_STREAM("Joint " << motor_ids[i] << ": " << std::abs(motorDate_recv[motor_ids[i]].current_));
    // }


    // 5Dof
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


    for(int motor_id : motor_ids) {
        ROS_INFO_STREAM("Torque from Current " <<motor_id<< ": ");
        ROS_INFO_STREAM(" "<< std::abs(motorDate_recv[motor_id].current_));

        double pos_des   = jointCommand_[motor_id].pos_des_;
        double pos_now   = jointCommand_[motor_id].pos_;
        double vel_now   = jointCommand_[motor_id].vel_;

        double pos_err = pos_des - pos_now;
        double torque_kp = joint_kps_[motor_id] * pos_err;
        double torque_kd = joint_kds_[motor_id] * vel_now;
        double torque_pd   = torque_kp - torque_kd;

        ROS_INFO_STREAM("Torque from PD formula " <<motor_id<< ": ");
        ROS_INFO_STREAM("torque kp :" << torque_kp << "  torque kd :" << torque_kd);
        ROS_INFO_STREAM("torque pd :" << torque_pd);
    }

    // ROS_INFO_STREAM("Torque from Current " <<12<< ": ");
    // ROS_INFO_STREAM(" "<< std::abs(motorDate_recv[12].current_));

    // double pos_des   = jointCommand_[12].pos_des_;
    // double pos_now   = jointCommand_[12].pos_;
    // double vel_now   = jointCommand_[12].vel_;

    // double pos_err = pos_des - pos_now;
    // double torque_kp = joint_kps_[12] * pos_err;
    // double torque_kd = joint_kds_[12] * vel_now;
    // double torque_pd   = torque_kp - torque_kd;

    // ROS_INFO_STREAM("Torque from PD formula " <<12<< ": ");
    // ROS_INFO_STREAM("torque kp :" << torque_kp << "  torque kd :" << torque_kd);
    // ROS_INFO_STREAM("torque pd :" << torque_pd);



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

    // if (right_solver) {
    //     // 建议限流打印，避免每个周期刷屏
    //     // 每 1 秒输出一次（你也可以改成 0.5、2.0）
    //     const double fake_t1    = 0.15;   // 假电机角度1（rad）
    //     const double fake_t2    = -0.10;  // 假电机角度2（rad）
    //     const double fake_t1dot = 0.80;   // 假电机速度1（rad/s）
    //     const double fake_t2dot = -0.40;  // 假电机速度2（rad/s）

    //     double tp = 0.0, tr = 0.0;
    //     bool fk_ok = right_solver->forwardKinematics(fake_t1, fake_t2, tp, tr);

    //     if (fk_ok) {
    //         double tpdot = 0.0, trdot = 0.0;
    //         bool vel_ok = right_solver->motorVelToJointVel(fake_t1, fake_t2, tp, tr,
    //                                                 fake_t1dot, fake_t2dot,
    //                                                 tpdot, trdot);

    //         // 有限差分验证：用小 dt 推一步电机角，再 FK，看 tp/tr 的变化率是否接近映射结果
    //         const double dt = 1e-5;
    //         double tp2 = 0.0, tr2 = 0.0;
    //         bool fk2_ok = right_solver->forwardKinematics(fake_t1 + fake_t1dot * dt,
    //                                                 fake_t2 + fake_t2dot * dt,
    //                                                 tp2, tr2);

    //         if (vel_ok && fk2_ok) {
    //             const double tpdot_fd = (tp2 - tp) / dt;
    //             const double trdot_fd = (tr2 - tr) / dt;

    //             ROS_INFO_STREAM_THROTTLE(1.0,
    //                 "[MechSolver FakeCheck] "
    //                 "motor(t1,t2)=(" << fake_t1 << "," << fake_t2 << "), "
    //                 "motorDot=(" << fake_t1dot << "," << fake_t2dot << ") | "
    //                 "FK(tp,tr)=(" << tp << "," << tr << ") | "
    //                 "Map(tpDot,trDot)=(" << tpdot << "," << trdot << ") | "
    //                 "FD(tpDot,trDot)=(" << tpdot_fd << "," << trdot_fd << ") | "
    //                 "Diff=(" << (tpdot - tpdot_fd) << "," << (trdot - trdot_fd) << ")"
    //             );
    //         } else {
    //             ROS_WARN_STREAM_THROTTLE(1.0,
    //                 "[MechSolver FakeCheck] velocity mapping or FK(perturbed) failed. "
    //                 "fk_ok=" << fk_ok << ", vel_ok=" << vel_ok << ", fk2_ok=" << fk2_ok
    //             );
    //         }
    //     } else {
    //         ROS_WARN_STREAM_THROTTLE(1.0, "[MechSolver FakeCheck] FK failed for fake inputs.");
    //     }
    // }
    // ==================== 假数据验证结束 ====================
}

void BotHW::write(ros::Time time, ros::Duration period) {
    if (!emergencyStopFlag_) {
        for (int motor_id = 0; motor_id < TOTAL_MOTORS; ++motor_id) {
            const double desired_pos = jointCommand_[motor_id].pos_des_;

            if (desired_pos < joint_limits_[motor_id].min_position ||
                desired_pos > joint_limits_[motor_id].max_position) {

                emergencyStopFlag_ = true;

                ROS_ERROR_STREAM(
                    "Reach Position Limit -> ENTER DAMPING BRAKE. motor_id=" << motor_id
                    << " desired=" << desired_pos
                    << " min=" << joint_limits_[motor_id].min_position
                    << " max=" << joint_limits_[motor_id].max_position
                );
                break;
            }
        }
    }

    for (int motor_id = 0; motor_id < TOTAL_MOTORS; ++motor_id) {

        if (emergencyStopFlag_) {
            mvrSendcmd_[motor_id].pos_des_ = jointCommand_[motor_id].pos_;
            mvrSendcmd_[motor_id].vel_des_ = 0.0;
            mvrSendcmd_[motor_id].kp_      = 0.0;
            mvrSendcmd_[motor_id].kd_      = 1.0; 
            mvrSendcmd_[motor_id].ff_      = 0.0;

        } else {
            mvrSendcmd_[motor_id].pos_des_ = jointCommand_[motor_id].pos_des_;
            mvrSendcmd_[motor_id].vel_des_ = 0.0;
            mvrSendcmd_[motor_id].kp_      = joint_kps_[motor_id];
            mvrSendcmd_[motor_id].kd_      = joint_kds_[motor_id];
            mvrSendcmd_[motor_id].ff_      = 0.0;
        }
    }

    EtherCAT_Send_Command_New(mvrSendcmd_);

    if (emergencyStopFlag_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[BotHW] emergencyStopFlag_=true, damping brake mode active.");
    }
}

void BotHW::odomCallback(const sensor_msgs::Imu::ConstPtr &odom) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    yesenceIMU_ = *odom;
}

void BotHW::commandCallback(const mvr_robot_control::ActionData::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    if (emergencyStopFlag_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "In damping brake mode, ignore incoming commands.");
        return;
    }

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
    // for (int i = 0; i < motor_ids.size(); ++i) {
    //     int motor_id = motor_ids[i];

    //     ROS_INFO_STREAM("  Joint " << motor_id << ": " << msg->joint_pos[i]);
        
    //     jointCommand_[motor_id].pos_des_ = msg->joint_pos[i];
    //     // jointCommand_[motor_id + 1].pos_des_ = -msg->joint_pos[i];
    // }


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

    // Whole Body 22Dof
    // if (!(left_solver && right_solver)) {
    //     ROS_ERROR_STREAM("MechanismSolver not initialized.");
    //     return;
    // }

    // jointCommand_[0].pos_des_ = msg->joint_pos[0];
    // jointCommand_[1].pos_des_ = msg->joint_pos[1];
    // jointCommand_[2].pos_des_ = msg->joint_pos[2];
    // jointCommand_[3].pos_des_ = msg->joint_pos[3];

    // {
    //     const double theta_p = msg->joint_pos[4];
    //     const double theta_r = msg->joint_pos[5];

    //     double theta1 = 0.0, theta2 = 0.0;
    //     if (left_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
    //         jointCommand_[4].pos_des_ = theta1;
    //         jointCommand_[5].pos_des_ = theta2;
    //     } else {
    //         ROS_WARN_STREAM("Left ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
    //     }
    // }

    // jointCommand_[6].pos_des_ = msg->joint_pos[6];
    // jointCommand_[7].pos_des_ = msg->joint_pos[7];
    // jointCommand_[8].pos_des_ = msg->joint_pos[8];
    // jointCommand_[9].pos_des_ = msg->joint_pos[9];

    // {
    //     const double theta_p = msg->joint_pos[10];
    //     const double theta_r = msg->joint_pos[11];

    //     double theta1 = 0.0, theta2 = 0.0;
    //     if (right_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
    //         jointCommand_[10].pos_des_ = theta1;  
    //         jointCommand_[11].pos_des_ = theta2;
    //     } else {
    //         ROS_WARN_STREAM("Right ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
    //     }
    // }

    // for (int msg_idx = 12; msg_idx <= 21; ++msg_idx) {
    //     const int motor_id = msg_idx; // 10->12, ..., 19->21
    //     jointCommand_[motor_id].pos_des_ = msg->joint_pos[msg_idx];
    // }

    // Whole Body 20dof
    // if (!(left_solver && right_solver)) {
    //     ROS_ERROR_STREAM("MechanismSolver not initialized.");
    //     return;
    // }

    // jointCommand_[0].pos_des_ = msg->joint_pos[0];
    // jointCommand_[1].pos_des_ = msg->joint_pos[1];
    // jointCommand_[2].pos_des_ = msg->joint_pos[2];
    // jointCommand_[3].pos_des_ = msg->joint_pos[3];

    // {
    //     const double theta_p = msg->joint_pos[4];
    //     const double theta_r = 0.0;

    //     double theta1 = 0.0, theta2 = 0.0;
    //     if (left_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
    //         jointCommand_[4].pos_des_ = theta1;
    //         jointCommand_[5].pos_des_ = theta2;
    //     } else {
    //         ROS_WARN_STREAM("Left ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
    //     }
    // }

    // jointCommand_[6].pos_des_ = msg->joint_pos[5];
    // jointCommand_[7].pos_des_ = msg->joint_pos[6];
    // jointCommand_[8].pos_des_ = msg->joint_pos[7];
    // jointCommand_[9].pos_des_ = msg->joint_pos[8];

    // {
    //     const double theta_p = msg->joint_pos[9];
    //     const double theta_r = 0.0;

    //     double theta1 = 0.0, theta2 = 0.0;
    //     if (right_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
    //         jointCommand_[10].pos_des_ = theta1;  
    //         jointCommand_[11].pos_des_ = theta2;
    //     } else {
    //         ROS_WARN_STREAM("Right ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
    //     }
    // }

    // for (int msg_idx = 10; msg_idx <= 19; ++msg_idx) {
    //     const int motor_id = msg_idx + 2; // 10->12, ..., 19->21
    //     jointCommand_[motor_id].pos_des_ = msg->joint_pos[msg_idx];
    // }

    // Ankle
    if (!(left_solver && right_solver)) {
        ROS_ERROR_STREAM("MechanismSolver not initialized.");
        return;
    }

    {
        const double theta_p = msg->joint_pos[0];
        const double theta_r = msg->joint_pos[1];

        double theta1 = 0.0, theta2 = 0.0;
        if (left_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
            jointCommand_[4].pos_des_ = theta1;
            jointCommand_[5].pos_des_ = theta2;
        } else {
            ROS_WARN_STREAM("Left ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
        }
    }


    {
        const double theta_p = msg->joint_pos[2];
        const double theta_r = msg->joint_pos[3];

        double theta1 = 0.0, theta2 = 0.0;
        if (right_solver->inverseKinematics(theta_p, theta_r, theta1, theta2)) {
            jointCommand_[10].pos_des_ = theta1;  
            jointCommand_[11].pos_des_ = theta2;
        } else {
            ROS_WARN_STREAM("Right ankle IK failed (tp=" << theta_p << ", tr=" << theta_r << "). Keep previous commands.");
        }
    }


}



