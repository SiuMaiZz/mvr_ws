/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 11:17:58
 * @LastEditTime: 2022-11-13 17:16:18
 */
#ifndef TRANSMIT_H
#define TRANSMIT_H

#define SLAVE_NUMBER 4

#define TOTAL_MOTORS 22

// 逐从站通道数：legs(6,6) + upper(5,5)
#define MOTORS_PER_SLAVE_0 6
#define MOTORS_PER_SLAVE_1 6
#define MOTORS_PER_SLAVE_2 5
#define MOTORS_PER_SLAVE_3 5

// 逐从站在全局数组中的起始索引（累加得到：0,6,12,17）
#define MOTOR_START_IDX_0 0
#define MOTOR_START_IDX_1 6
#define MOTOR_START_IDX_2 12
#define MOTOR_START_IDX_3 17

// 可选：若存在 IMU，从站索引（无 IMU 可设为 -1）
#ifndef IMU_SLAVE_INDEX
#define IMU_SLAVE_INDEX 4
#endif

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sys/time.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  double pos_, vel_, tau_, current_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
} YKSMotorData;
extern YKSMotorData motorDate_recv[TOTAL_MOTORS];
typedef struct
{
  float angle_float[3];
  float gyro_float[3];
  float accel_float[3];
  float mag_float[3];
  float quat_float[4];
} YKSIMUData;
extern YKSIMUData imuData_recv;
int EtherCAT_Init(char* ifname);
void EtherCAT_Run();
void EtherCAT_Command_Set();
void EtherCAT_Send_Command(YKSMotorData* data);
void EtherCAT_Send_Command_New(YKSMotorData* mot_data);
void EtherCAT_Send_Command_Position(YKSMotorData* data);
void EtherCAT_Send_Command_Speed(float* data);
void EtherCAT_Get_State();
void EtherCAT_Get_State_New();
void Revert_State(YKSMotorData* motor_data);
void startRun();

#define MOTORS_PER_SLAVE(slave) \
  ((slave)==0 ? MOTORS_PER_SLAVE_0 : \
  ((slave)==1 ? MOTORS_PER_SLAVE_1 : \
  ((slave)==2 ? MOTORS_PER_SLAVE_2 : \
  ((slave)==3 ? MOTORS_PER_SLAVE_3 : 0))))

#define MOTOR_START_IDX(slave) \
  ((slave)==0 ? MOTOR_START_IDX_0 : \
  ((slave)==1 ? MOTOR_START_IDX_1 : \
  ((slave)==2 ? MOTOR_START_IDX_2 : \
  ((slave)==3 ? MOTOR_START_IDX_3 : 0))))

#define GLOBAL_IDX(slave, ch) (MOTOR_START_IDX(slave) + (ch))

// 判断是否腿部从站（索引 0/1），用于扭矩比例选择等
#define IS_LEG_SLAVE(slave) ((slave) < 2)

// 判断“膝/踝”通道（沿用原规则：ch==2 或 ch==3）
#define IS_KNEE_OR_HIP_CH(ch) ((ch)==0 || (ch)==1 ||(ch)==2 || (ch)==3)


#ifdef __cplusplus
};
#endif

#endif  // PROJECT_RT_ETHERCAT_H