#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <atomic>
#include <array>
#include <cmath>
#include <mutex>
#include <pthread.h>

#include "mvr_robot_control/MechanismSolver.h"
#include "command.h"   // 你原脚本里 EtherCAT/数据结构的头文件

static constexpr int L_M1 = 4, L_M2 = 5;
static constexpr int R_M1 = 10, R_M2 = 11;

static YKSMotorData Sendcmd_[TOTAL_MOTORS];

static std::mutex cmd_mtx;
static double cmd_tpL = 0.0, cmd_trL = 0.0, cmd_tpR = 0.0, cmd_trR = 0.0;
static std::atomic<bool> has_cmd{false};

static ros::Publisher joint_pub;

// 安全初始化：必须把 22 路命令都写一遍，避免下发垃圾值
static void initSafeCmd() {
  for (int i = 0; i < TOTAL_MOTORS; ++i) {
    Sendcmd_[i].pos_des_ = 0.0;
    Sendcmd_[i].vel_des_ = 0.0;
    Sendcmd_[i].kp_ = 0.0;
    Sendcmd_[i].kd_ = 0.0;
    Sendcmd_[i].ff_ = 0.0;
  }

  // 只给脚踝打开增益（按你需要调整）
  Sendcmd_[L_M1].kp_ = 10.0; Sendcmd_[L_M1].kd_ = 0.5;
  Sendcmd_[L_M2].kp_ = 10.0; Sendcmd_[L_M2].kd_ = 0.5;
  Sendcmd_[R_M1].kp_ = 10.0; Sendcmd_[R_M1].kd_ = 0.5;
  Sendcmd_[R_M2].kp_ = 10.0; Sendcmd_[R_M2].kd_ = 0.5;
}

static void publishJointState() {
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.name.reserve(TOTAL_MOTORS);
  js.position.reserve(TOTAL_MOTORS);
  js.velocity.reserve(TOTAL_MOTORS);
  js.effort.reserve(TOTAL_MOTORS);

  for (int i = 0; i < TOTAL_MOTORS; ++i) {
    js.name.push_back("joint_" + std::to_string(i+1));
    js.position.push_back(motorDate_recv[i].pos_);
    js.velocity.push_back(motorDate_recv[i].vel_);
    js.effort.push_back(motorDate_recv[i].tau_);
  }
  joint_pub.publish(js);
}

static void ankleCmdCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 4) {
    ROS_WARN_STREAM("ankleCmdCb: need 4 values [tpL,trL,tpR,trR], got " << msg->data.size());
    return;
  }
  std::lock_guard<std::mutex> lk(cmd_mtx);
  cmd_tpL = msg->data[0];
  cmd_trL = msg->data[1];
  cmd_tpR = msg->data[2];
  cmd_trR = msg->data[3];
  has_cmd.store(true);

  ROS_INFO_STREAM_THROTTLE(0.5,
    "[ankle_control_node] RX /ankle_cmd "
    "tpL=" << cmd_tpL << " trL=" << cmd_trL
    << " | tpR=" << cmd_tpR << " trR=" << cmd_trR);

}

// 控制循环（持续 Get/Send）
static void controlLoop(MechanismSolver* left_solver, MechanismSolver* right_solver) {
  ros::Rate rate(50); // 200Hz，按你的总线能力调

  // 预热：先 hold 当前位姿一段时间，避免突然拉扯
  const int WARMUP_FRAMES = 50;
  int warmup_count = 0;

  while (ros::ok()) {
    EtherCAT_Get_State_New();

    if (warmup_count < WARMUP_FRAMES) {
      for (int i = 0; i < TOTAL_MOTORS; ++i) {
        Sendcmd_[i].pos_des_ = motorDate_recv[i].pos_;
        Sendcmd_[i].vel_des_ = 0.0;
        Sendcmd_[i].ff_ = 0.0;
        // kp/kd 你也可以在 warmup 期间置 0
      }
      EtherCAT_Send_Command_New((YKSMotorData*)Sendcmd_);
      warmup_count++;
      publishJointState();
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    // 读取最新目标（不阻塞控制循环）
    double tpL, trL, tpR, trR;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx);
      tpL = cmd_tpL; trL = cmd_trL; tpR = cmd_tpR; trR = cmd_trR;
    }

    // 若尚未收到命令，则默认保持当前位姿（避免乱动）
    if (!has_cmd.load()) {
      for (int i = 0; i < TOTAL_MOTORS; ++i) {
        Sendcmd_[i].pos_des_ = motorDate_recv[i].pos_;
        Sendcmd_[i].vel_des_ = 0.0;
        Sendcmd_[i].ff_ = 0.0;
      }
      EtherCAT_Send_Command_New((YKSMotorData*)Sendcmd_);
      publishJointState();
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    // ---------- 脚踝 IK：tp,tr -> (t1,t2) ----------
    double t1L = 0.0, t2L = 0.0, t1R = 0.0, t2R = 0.0;
    bool okL = left_solver->inverseKinematics(tpL, trL, t1L, t2L);
    bool okR = right_solver->inverseKinematics(tpR, trR, t1R, t2R);

    if (okL) {
      Sendcmd_[L_M1].pos_des_ = t1L;
      Sendcmd_[L_M2].pos_des_ = t2L;
      Sendcmd_[L_M1].vel_des_ = 0.0;
      Sendcmd_[L_M2].vel_des_ = 0.0;
      Sendcmd_[L_M1].ff_ = 0.0;
      Sendcmd_[L_M2].ff_ = 0.0;
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "Left ankle IK failed: tp=" << tpL << ", tr=" << trL);
      // 失败时保持现状
      Sendcmd_[L_M1].pos_des_ = motorDate_recv[L_M1].pos_;
      Sendcmd_[L_M2].pos_des_ = motorDate_recv[L_M2].pos_;
    }

    if (okR) {
      Sendcmd_[R_M1].pos_des_ = t1R;
      Sendcmd_[R_M2].pos_des_ = t2R;
      Sendcmd_[R_M1].vel_des_ = 0.0;
      Sendcmd_[R_M2].vel_des_ = 0.0;
      Sendcmd_[R_M1].ff_ = 0.0;
      Sendcmd_[R_M2].ff_ = 0.0;
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "Right ankle IK failed: tp=" << tpR << ", tr=" << trR);
      Sendcmd_[R_M1].pos_des_ = motorDate_recv[R_M1].pos_;
      Sendcmd_[R_M2].pos_des_ = motorDate_recv[R_M2].pos_;
    }

    // 关键：每周期都要 Send
    EtherCAT_Send_Command_New((YKSMotorData*)Sendcmd_);

    // 可选：打印 FK 验证（限流）
    {
      double tp_fk=0, tr_fk=0;
      if (left_solver->forwardKinematics(motorDate_recv[L_M1].pos_, motorDate_recv[L_M2].pos_, tp_fk, tr_fk)) {
        ROS_INFO_STREAM_THROTTLE(1.0, "[FK L] tp=" << tp_fk << " tr=" << tr_fk);
      }
    }

    ROS_INFO_STREAM_THROTTLE(0.5,
        "[ankle_control_node] IK target motor "
        "L(4,5)=(" << Sendcmd_[L_M1].pos_des_ << "," << Sendcmd_[L_M2].pos_des_ << ") "
        "R(10,11)=(" << Sendcmd_[R_M1].pos_des_ << "," << Sendcmd_[R_M2].pos_des_ << ")");


    publishJointState();
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ankle_control_node");
  ros::NodeHandle nh;

  joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
  ros::Subscriber sub = nh.subscribe("/ankle_cmd", 10, ankleCmdCb);

  // 机构参数（也可以从 rosparam 读）
  double l1 = 47.5, d1 = 26.0, h1 = 213.0, h2 = 149.0;

  MechanismSolver left_solver(l1, d1, h1, h2, MechanismSolver::Side::Left);
  MechanismSolver right_solver(l1, d1, h1, h2, MechanismSolver::Side::Right);

  EtherCAT_Init(const_cast<char*>("enp4s0"));
  ROS_INFO("[ankle_control_node] EtherCAT init OK, start control loop.");
  initSafeCmd();

  // 用 pthread 起实时循环线程（和你原脚本一致）
  pthread_t control_thread;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  sched_param sched; sched.sched_priority = 50;
  pthread_attr_setschedparam(&attr, &sched);

  if (pthread_create(&control_thread, &attr, [](void* arg)->void* {
        auto* p = reinterpret_cast<std::pair<MechanismSolver*,MechanismSolver*>*>(arg);
        controlLoop(p->first, p->second);
        return nullptr;
      }, new std::pair<MechanismSolver*,MechanismSolver*>(&left_solver, &right_solver)) != 0) {
    ROS_ERROR("Failed to create control loop thread");
    return -1;
  }

  pthread_join(control_thread, nullptr);
  return 0;
}
