#include "mvr_robot_control/StateVizPublisher.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mvr_robot_control::state_estimation {

StateVizPublisher::StateVizPublisher(ros::NodeHandle& nh) {
  ros::NodeHandle pnh("~");

  // joint_names: 必须与 URDF 中 joint name 严格一致，且长度=22
  if (!pnh.getParam("joint_names", joint_names_) || joint_names_.size() != 22) {
    ROS_FATAL("~joint_names must exist and have size 22 (URDF joint names in ObserveData index order).");
    ros::shutdown();
    return;
  }

  pnh.param("odom_frame", odom_frame_, odom_frame_);
  pnh.param("base_frame", base_frame_, base_frame_);
  pnh.param("publish_rate", publish_rate_, publish_rate_);
  pnh.param("base_height_guess", base_height_guess_, base_height_guess_);
  pnh.param("quat_xyzw", quat_xyzw_, quat_xyzw_);
  pnh.param("publish_tf", publish_tf_, publish_tf_);
  pnh.param("publish_odom", publish_odom_, publish_odom_);

  sub_observe_ = nh.subscribe("/observe_data", 50, &StateVizPublisher::observeCb, this);
  pub_joint_states_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 50);
  pub_odom_ = nh.advertise<nav_msgs::Odometry>("/odom", 50);

  const double dt = (publish_rate_ > 1e-3) ? (1.0 / publish_rate_) : 0.005;
  timer_ = nh.createTimer(ros::Duration(dt), &StateVizPublisher::timerCb, this);

  ROS_INFO_STREAM("StateVizPublisher started. publish_rate=" << publish_rate_
                  << " odom_frame=" << odom_frame_ << " base_frame=" << base_frame_);
}

void StateVizPublisher::observeCb(const mvr_robot_control::ObserveDataConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  last_obs_ = *msg;
  have_obs_ = true;
}

void StateVizPublisher::timerCb(const ros::TimerEvent&) {
  mvr_robot_control::ObserveData obs;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!have_obs_) return;
    obs = last_obs_;
  }

  const ros::Time stamp = obs.header.stamp.isZero() ? ros::Time::now() : obs.header.stamp;

  publishJointStates(obs, stamp);
  publishOdomAndTf(obs, stamp);
}

void StateVizPublisher::publishJointStates(const mvr_robot_control::ObserveData& obs, const ros::Time& stamp) {
  sensor_msgs::JointState js;
  js.header.stamp = stamp;
  js.name = joint_names_;
  js.position.resize(22);
  js.velocity.resize(22);

  for (int i = 0; i < 22; ++i) {
    js.position[i] = obs.joint_pos[i];
    js.velocity[i] = obs.joint_vel[i];
  }
  pub_joint_states_.publish(js);
}

void StateVizPublisher::publishOdomAndTf(const mvr_robot_control::ObserveData& obs, const ros::Time& stamp) {
  // 四元数顺序：用参数控制
  tf2::Quaternion q;
  if (quat_xyzw_) {
    q.setX(obs.quat_float[0]);
    q.setY(obs.quat_float[1]);
    q.setZ(obs.quat_float[2]);
    q.setW(obs.quat_float[3]);
  } else {
    q.setW(obs.quat_float[0]);
    q.setX(obs.quat_float[1]);
    q.setY(obs.quat_float[2]);
    q.setZ(obs.quat_float[3]);
  }
  q.normalize();

  // 最小方案：base 位置固定；后续你可以用 Pinocchio/接触约束估计 x,y,z
  const double x = 0.0, y = 0.0, z = base_height_guess_;

  if (publish_tf_) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_br_.sendTransform(tf);
  }

  if (publish_odom_) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // 最小可用：角速度来自 ObserveData.imu_angular_vel；线速度先填 0
    odom.twist.twist.angular.x = obs.imu_angular_vel[0];
    odom.twist.twist.angular.y = obs.imu_angular_vel[1];
    odom.twist.twist.angular.z = obs.imu_angular_vel[2];

    pub_odom_.publish(odom);
  }
}

}  // namespace mvr_robot_control::state_estimation