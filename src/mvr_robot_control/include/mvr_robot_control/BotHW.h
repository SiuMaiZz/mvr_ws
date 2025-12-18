//-----------------------------------------------------------------------------
// File: BotHW.h
// Author: Akisora
// Created: 2025-11-25
//-----------------------------------------------------------------------------

#ifndef BOT_HW_H
#define BOT_HW_H

#define TOTAL_MOTORS 22

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include "command.h"
#include "transmit.h"
#include "mvr_robot_control/MechanismSolver.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mvr_robot_control/ObserveData.h>
#include <mvr_robot_control/ActionData.h>
#include <mutex>
#include <vector>
#include <XmlRpcValue.h>

struct MotorData
{
    double pos_, vel_, tau_;
    double pos_des_, vel_des_, kp_, kd_, ff_;
};

struct ImuData
{
    double ori[4];
    double ori_cov[9];
    double angular_vel[3];
    double angular_vel_cov[9];
    double linear_acc[3];
    double linear_acc_cov[9];
};

class BotHW : public hardware_interface::RobotHW {
public:
    BotHW() = default;;
    virtual ~BotHW() = default;;

    ros::Publisher observe_pub_;

    std::vector<joint_limits_interface::JointLimits> joint_limits_;

    bool init(ros::NodeHandle& nh);

    void read(ros::Time time, ros::Duration period);

    void write(ros::Time time, ros::Duration period);

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface joint_position_interface_;

    std::vector<double> default_joint_positions_;

    std::vector<double> joint_kps_;
    std::vector<double> joint_kds_;
    
    MotorData jointCommand_[TOTAL_MOTORS]{};

    ImuData imuData_{};

    YKSMotorData mvrSendcmd_[TOTAL_MOTORS];
    YKSMotorData mvrSendDefaultcmd_[TOTAL_MOTORS];

    sensor_msgs::Imu yesenceIMU_;
    ros::Subscriber imu_sub_;
    ros::Subscriber command_sub_;

    std::mutex imu_mutex_;
    std::mutex cmd_mutex_;

    std::unique_ptr<MechanismSolver> solver;

    void odomCallback(const sensor_msgs::Imu::ConstPtr &odom);
    void commandCallback(const mvr_robot_control::ActionData::ConstPtr &msg);

    const double l1 = 47.5;
    const double d1 = 26.0;
    const double h1 = 213.0;
    const double h2 = 149.0;
};


#endif