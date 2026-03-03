//-----------------------------------------------------------------------------
// File: StateVizPublisher.h
// Author: Akisora
// Created: 2026-03-02
//-----------------------------------------------------------------------------

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <vector>
#include <string>

#include <mvr_robot_control/ObserveData.h>

namespace mvr_robot_control::state_estimation {

class StateVizPublisher {
public:
  explicit StateVizPublisher(ros::NodeHandle& nh);

private:
  void observeCb(const mvr_robot_control::ObserveDataConstPtr& msg);
  void timerCb(const ros::TimerEvent&);

  void publishJointStates(const mvr_robot_control::ObserveData& obs, const ros::Time& stamp);
  void publishOdomAndTf(const mvr_robot_control::ObserveData& obs, const ros::Time& stamp);

private:
  // ROS I/O
  ros::Subscriber sub_observe_;
  ros::Publisher pub_joint_states_;
  ros::Publisher pub_odom_;
  tf2_ros::TransformBroadcaster tf_br_;
  ros::Timer timer_;

  // Params
  std::vector<std::string> joint_names_;
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  double publish_rate_{200.0};      // 可视化/里程计发布频率
  double base_height_guess_{0.9};   // 先固定高度；后续可用 Pinocchio 修正
  bool quat_xyzw_{true};            // ObserveData.quat_float 是否为 [x,y,z,w]
  bool publish_tf_{true};
  bool publish_odom_{true};

  // Latest data buffer
  std::mutex mtx_;
  bool have_obs_{false};
  mvr_robot_control::ObserveData last_obs_;
};

}  // namespace mvr_robot_control::state_estimation