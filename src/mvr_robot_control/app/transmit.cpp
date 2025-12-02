extern "C" {
#include "ethercat.h"  
#include "motor_control.h"
#include "transmit.h"
}
#include <iostream>
#include "queue.h"
#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#define EC_TIMEOUTM

spsc_queue<EtherCAT_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];
std::atomic<bool> running{ false };
std::thread runThread;
YKSMotorData motorDate_recv[TOTAL_MOTORS];
YKSIMUData imuData_recv;
char IOmap[4096];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
uint64_t num;
bool isConfig[SLAVE_NUMBER]{ false };

#define EC_TIMEOUTMON 500

void EtherCAT_Data_Get();

void EtherCAT_Command_Set();
void EtherCAT_Get_State();
void EtherCAT_Get_State_New();
void EtherCAT_Send_Command(YKSMotorData* data);
void EtherCAT_Send_Command_New(YKSMotorData* mot_data);
void EtherCAT_Send_Command_Position(YKSMotorData* data);
void EtherCAT_Send_Command_Speed(float* data);
void Revert_State(YKSMotorData* motor_data);

static const uint8_t kChannelMapLegs[6]     = {1, 2, 3, 4, 5, 6};
static const uint8_t kChannelMapUpper[5]    = {1, 2, 4, 5, 6};

static void degraded_handler()
{
  printf("[EtherCAT Error] Logging error...\n");
  time_t current_time = time(NULL);
  char* time_str = ctime(&current_time);
  printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
  printf("[EtherCAT Error] Stopping RT process.\n");
}

static int run_ethercat(const char* ifname)
{
  int i;
  int oloop, iloop, chk;
  needlf = FALSE;
  inOP = FALSE;

  num = 1;

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname))
  {
    printf("[EtherCAT Init] Initialization on device %s succeeded.\n", ifname);
    /* find and auto-config slaves */

    if (ec_config_init(FALSE) > 0)
    {
      printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
      if (ec_slavecount < SLAVE_NUMBER)
      {
        printf("[RT EtherCAT] Warning: Expected %d slaves, found %d.\n", SLAVE_NUMBER, ec_slavecount);
      }

      for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
        ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

      ec_config_map(&IOmap);
      for (int i = 1; i <= ec_slavecount; i++) {
        printf("[Init] Slave %d: Obytes=%d, Ibytes=%d, sizeof(EtherCAT_Msg)=%zu\n",
               i, ec_slave[i].Obytes, ec_slave[i].Ibytes, sizeof(EtherCAT_Msg));
      }

      ec_configdc();

      printf("[EtherCAT Init] Mapped slaves.\n");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

      for (int slave_idx = 0; slave_idx <= ec_slavecount; slave_idx++)
      {
        printf("[SLAVE %d]\n", slave_idx);
        printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
        printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
        printf("\n");
      }

      oloop = ec_slave[0].Obytes;
      if ((oloop == 0) && (ec_slave[0].Obits > 0))
        oloop = 1;
      if (oloop > 8)
        oloop = 8;
      iloop = ec_slave[0].Ibytes;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0))
        iloop = 1;
      if (iloop > 8)
        iloop = 8;

      printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0],
             ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

      printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      chk = 40;
      /* wait for all slaves to reach OP state */
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

      if (ec_slave[0].state == EC_STATE_OPERATIONAL)
      {
        printf("[EtherCAT Init] Operational state reached for all slaves.\n");
        inOP = TRUE;
        return 1;
      }
      else
      {
        printf("[EtherCAT Error] Not all slaves reached operational state.\n");
        ec_readstate();
        for (i = 1; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state,
                   ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
    }
    else
    {
      printf("[EtherCAT Error] No slaves found!\n");
    }
  }
  else
  {
    printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n", ifname);
  }
  return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

static OSAL_THREAD_FUNC ecatcheck(void* ptr)
{
  (void)ptr;
  int slave = 0;
  while (1)
  {
    // count errors
    if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
      err_iteration_count = 0;
      err_count = 0;
    }

    if (err_count > K_ETHERCAT_ERR_MAX)
    {
      // possibly shut down
      printf("[EtherCAT Error] EtherCAT connection degraded.\n");
      printf("[Simulink-Linux] Shutting down....\n");
      degraded_handler();
      break;
    }
    err_iteration_count++;

    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
      if (needlf)
      {
        needlf = FALSE;
        printf("\n");
      }
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++)
      {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
            err_count++;
          }
          else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
          {
            printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
            err_count++;
          }
          else if (ec_slave[slave].state > 0)
          {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
            }
          }
          else if (!ec_slave[slave].islost)
          {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[slave].state)
            {
              ec_slave[slave].islost = TRUE;
              printf("[EtherCAT Error] Slave %d lost\n", slave);
              err_count++;
            }
          }
        }
        if (ec_slave[slave].islost)
        {
          if (!ec_slave[slave].state)
          {
            if (ec_recover_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("[EtherCAT Status] Slave %d recovered\n", slave);
            }
          }
          else
          {
            ec_slave[slave].islost = FALSE;
            printf("[EtherCAT Status] Slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate)
        printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(50000);
  }
}

int EtherCAT_Init(char* ifname)
{
  int i;
  int rc;
  printf("[EtherCAT] Initializing EtherCAT\n");
  osal_thread_create((void*)&checkThread, 128000, (void*)&ecatcheck, (void*)&ctime);
  for (i = 1; i < 10; i++)
  {
    printf("[EtherCAT] Attempting to start EtherCAT, try %d of 10.\n", i);
    rc = run_ethercat(ifname);
    if (rc)
      break;
    osal_usleep(1000000);
  }
  if (rc)
    printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
  else
  {
    printf("[EtherCAT Error] Failed to initialize EtherCAT after 10 tries. \n");
  }
  return ec_slavecount;
}

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

EtherCAT_Msg Rx_Message[SLAVE_NUMBER];
EtherCAT_Msg Tx_Message[SLAVE_NUMBER];

void EtherCAT_Run()
{
  if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
  {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if (wkc_err_count > K_ETHERCAT_ERR_MAX)
  {
    printf("[EtherCAT Error] Error count too high!\n");
    degraded_handler();
  }
  // send
  EtherCAT_Command_Set();
  ec_send_processdata();
  // receive
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  EtherCAT_Data_Get();
  //  check for dropped packet
  if (wkc < expectedWKC)
  {
    printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  }
  else
  {
    needlf = TRUE;
  }
  wkc_err_iteration_count++;
}

void EtherCAT_Data_Get()
{
  for (int slave = 0; slave < ec_slavecount; ++slave)
  {
    EtherCAT_Msg* slave_src = (EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    if (slave_src)
      Rx_Message[slave] = *(EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
  }
}
void Revert_State(YKSMotorData* motor_data)
{
  YKSMotorData tmp;

  for (int i = 3; i < 6; i++)
  {
    tmp = motor_data[i];
    motor_data[i] = motor_data[i + 6];  // 0-3
    motor_data[i + 6] = tmp;
  }
}

// *****************************
// *****************************
// *****************************
/*函数功能：获取EtherCAT总线上从设备的状态信息*/
void EtherCAT_Get_State()
{
  wkc = ec_receive_processdata(EC_TIMEOUTRET);

  for (int slave = 0; slave < ec_slavecount; ++slave)
  {
    EtherCAT_Msg* slave_src = (EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    if (slave_src)
    {
      Rx_Message[slave] = *(EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    }
    if (slave < 2)
      RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
    else
      RV_can_imu_data_repack(&Rx_Message[slave]);
    if (slave == 0)
    {
      for (int motor_index = 0; motor_index < 6; motor_index++)
      {
        motorDate_recv[motor_index].pos_ = rv_motor_msg[motor_index].angle_actual_rad;
        motorDate_recv[motor_index].vel_ = rv_motor_msg[motor_index].speed_actual_rad;
        if (motor_index == 2 || motor_index == 3)
        {
          motorDate_recv[motor_index].tau_ = rv_motor_msg[motor_index].current_actual_float * 2.1;
        }
        else
        {
          motorDate_recv[motor_index].tau_ = rv_motor_msg[motor_index].current_actual_float * 1.4;
        }
      }
    }
    else if (slave == 1)
    {
      for (int motor_index = 0; motor_index < 6; motor_index++)
      {
        motorDate_recv[motor_index + 6].pos_ = rv_motor_msg[motor_index].angle_actual_rad;
        motorDate_recv[motor_index + 6].vel_ = rv_motor_msg[motor_index].speed_actual_rad;
        if (motor_index == 2 || motor_index == 3)
        {
          motorDate_recv[motor_index + 6].tau_ = rv_motor_msg[motor_index].current_actual_float * 2.1;
        }
        else
        {
          motorDate_recv[motor_index + 6].tau_ = rv_motor_msg[motor_index].current_actual_float * 1.4;
        }
      }
    }
    else if (slave == 2)
    {
      memcpy(imuData_recv.angle_float, imu_msg.angle_float, 3 * 4);
      memcpy(imuData_recv.gyro_float, imu_msg.gyro_float, 3 * 4);
      memcpy(imuData_recv.accel_float, imu_msg.accel_float, 3 * 4);
      memcpy(imuData_recv.mag_float, imu_msg.mag_float, 3 * 4);
      memcpy(imuData_recv.quat_float, imu_msg.quat_float, 4 * 4);
    }

    for (int motor_id = 0; motor_id < 6; motor_id++)
    {
      rv_motor_msg[motor_id].angle_actual_rad = 0;
    }
  }

  //  check for dropped packet
  if (wkc < expectedWKC)
  {
    printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  }
  else
  {
    needlf = TRUE;
  }
  wkc_err_iteration_count++;
}

void EtherCAT_Get_State_New()
{
  wkc = ec_receive_processdata(EC_TIMEOUTRET);

  for (int slave = 0; slave < ec_slavecount; ++slave)
  {
    // 读入 PDO 输入
    EtherCAT_Msg* src = (EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
    if (src) {
      Rx_Message[slave] = *src;
    }

    // 根据从站类型 repack
    if (slave < SLAVE_NUMBER) {
      // 电机从站
      RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
    }
#if (IMU_SLAVE_INDEX >= 0)
    else if (slave == IMU_SLAVE_INDEX) {
      // IMU 从站
      RV_can_imu_data_repack(&Rx_Message[slave]);
    }
#endif

    // 电机数据分拣到全局 motorDate_recv[0..TOTAL_MOTORS-1]
    if (slave < SLAVE_NUMBER) {
      const int ch_cnt   = MOTORS_PER_SLAVE(slave);
      const int start_id = MOTOR_START_IDX(slave);

      for (int ch = 0; ch < ch_cnt; ++ch) {
        const int gid = GLOBAL_IDX(slave, ch);   // 全局电机索引 0..21

        motorDate_recv[gid].pos_ = rv_motor_msg[ch].angle_actual_rad;
        motorDate_recv[gid].vel_ = rv_motor_msg[ch].speed_actual_rad;

        // 扭矩比例：腿部从站(0/1)的 ch==0/1/2/3 -> 2.1，其余 1.4；上半身(2/3) 全部 1.4
        const float scale =
            (IS_LEG_SLAVE(slave) && IS_KNEE_OR_HIP_CH(ch)) ? 2.1f : 1.4f;
        motorDate_recv[gid].tau_ = rv_motor_msg[ch].current_actual_float * scale;
        motorDate_recv[gid].current_ = rv_motor_msg[ch].current_actual_float;
      }
    }
#if (IMU_SLAVE_INDEX >= 0)
    else if (slave == IMU_SLAVE_INDEX) {
      // 拷贝 IMU 数据
      memcpy(imuData_recv.angle_float, imu_msg.angle_float, 3 * sizeof(float));
      memcpy(imuData_recv.gyro_float,  imu_msg.gyro_float,  3 * sizeof(float));
      memcpy(imuData_recv.accel_float, imu_msg.accel_float, 3 * sizeof(float));
      memcpy(imuData_recv.mag_float,   imu_msg.mag_float,   3 * sizeof(float));
      memcpy(imuData_recv.quat_float,  imu_msg.quat_float,  4 * sizeof(float));
    }
#endif

    // 清掉 6 通道的复用缓冲（硬件协议保持 6 通道）
    for (int ch = 0; ch < 6; ++ch) {
      rv_motor_msg[ch].angle_actual_rad     = 0;
      rv_motor_msg[ch].speed_actual_rad     = 0;
      rv_motor_msg[ch].current_actual_float = 0;
    }
  }

  // WKC 检查
  if (wkc < expectedWKC) {
    printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  } else {
    needlf = TRUE;
  }
  wkc_err_iteration_count++;
}

void EtherCAT_Send_Command(YKSMotorData* mot_data)
{
  uint16_t slave;

  if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
  {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if (wkc_err_count > K_ETHERCAT_ERR_MAX)
  {
    printf("[EtherCAT Error] Error count too high!\n");
    degraded_handler();
  }
  for (int index = 0; index < 12; index++)
  {
    if (index == 0)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 1, 1, mot_data[0].kp_, mot_data[0].kd_, mot_data[0].pos_des_,
                          mot_data[0].vel_des_, mot_data[0].ff_);
    }
    else if (index == 1)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 2, 2, mot_data[1].kp_, mot_data[1].kd_, mot_data[1].pos_des_,
                          mot_data[1].vel_des_, mot_data[1].ff_);
    }
    else if (index == 2)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 3, 3, mot_data[2].kp_, mot_data[2].kd_, mot_data[2].pos_des_,
                          mot_data[2].vel_des_, mot_data[2].ff_);
    }
    else if (index == 3)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 4, 4, mot_data[3].kp_, mot_data[3].kd_, mot_data[3].pos_des_,
                          mot_data[3].vel_des_, mot_data[3].ff_);
    }
    else if (index == 4)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 5, 5, mot_data[4].kp_, mot_data[4].kd_, mot_data[4].pos_des_,
                          mot_data[4].vel_des_, mot_data[4].ff_);
    }
    else if (index == 5)
    {
      slave = 0;
      send_motor_ctrl_cmd(&Tx_Message[slave], 6, 6, mot_data[5].kp_, mot_data[5].kd_, mot_data[5].pos_des_,
                          mot_data[5].vel_des_, mot_data[5].ff_);
    }
    else if (index == 6)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 1, 1, mot_data[6].kp_, mot_data[6].kd_, mot_data[6].pos_des_,
                          mot_data[6].vel_des_, mot_data[6].ff_);
    }
    else if (index == 7)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 2, 2, mot_data[7].kp_, mot_data[7].kd_, mot_data[7].pos_des_,
                          mot_data[7].vel_des_, mot_data[7].ff_);
    }
    else if (index == 8)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 3, 3, mot_data[8].kp_, mot_data[8].kd_, mot_data[8].pos_des_,
                          mot_data[8].vel_des_, mot_data[8].ff_);
    }
    else if (index == 9)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 4, 4, mot_data[9].kp_, mot_data[9].kd_, mot_data[9].pos_des_,
                          mot_data[9].vel_des_, mot_data[9].ff_);
    }
    else if (index == 10)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 5, 5, mot_data[10].kp_, mot_data[10].kd_, mot_data[10].pos_des_,
                          mot_data[10].vel_des_, mot_data[10].ff_);
    }
    else if (index == 11)
    {
      slave = 1;
      send_motor_ctrl_cmd(&Tx_Message[slave], 6, 6, mot_data[11].kp_, mot_data[11].kd_, mot_data[11].pos_des_,
                          mot_data[11].vel_des_, mot_data[11].ff_);
    }

    if (index == 5 || index == 11)
    {
      EtherCAT_Msg* slave_dest = (EtherCAT_Msg*)(ec_slave[slave + 1].outputs);
      if (slave_dest)
        *(EtherCAT_Msg*)(ec_slave[slave + 1].outputs) = Tx_Message[slave];
    }
  }

  ec_send_processdata();

}

void EtherCAT_Send_Command_New(YKSMotorData* mot_data)
{
  // 错误计数器维护
  if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD) {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if (wkc_err_count > K_ETHERCAT_ERR_MAX) {
    printf("[EtherCAT Error] Error count too high!\n");
    degraded_handler();
  }

  // 逐从站打包
  for (uint16_t slave = 0; slave < SLAVE_NUMBER; ++slave)
  {
    const int ch_cnt   = MOTORS_PER_SLAVE(slave);  // 0/1:6, 2/3:5
    const int start_id = MOTOR_START_IDX(slave);

    Tx_Message[slave].motor_num = ch_cnt;
    Tx_Message[slave].can_ide   = 0;

    // 选择该从站的 channel 映射表
    const uint8_t* chmap = nullptr;
    if (slave < 2) {          // legs
      chmap = kChannelMapLegs;   // 长度 6
    } else {                  // upper body
      chmap = kChannelMapUpper;  // 长度 5（跳过3）
    }

    for (int local = 0; local < ch_cnt; ++local) {
      const int      gid       = start_id + local;          
      const uint8_t  channel   = chmap[local];             
      const uint16_t motor_id  = static_cast<uint16_t>(local + 1); 

      send_motor_ctrl_cmd(&Tx_Message[slave],
                          channel, motor_id,
                          mot_data[gid].kp_,
                          mot_data[gid].kd_,
                          mot_data[gid].pos_des_,
                          mot_data[gid].vel_des_,
                          mot_data[gid].ff_);
    }

    // 写入从站 PDO 输出区
    EtherCAT_Msg* dst = (EtherCAT_Msg*)(ec_slave[slave + 1].outputs);
    if (dst) {
      const size_t need  = sizeof(EtherCAT_Msg);
      const int    avail = ec_slave[slave + 1].Obytes;
      if (avail >= (int)need) {
        *dst = Tx_Message[slave];
      } else if (avail > 0) {
        memcpy(dst, &Tx_Message[slave], (size_t)avail);
      }
    }
  }

  // IMU 从站（若存在）不发送控制帧
  ec_send_processdata();
}


void EtherCAT_Command_Set()
{
  for (int slave = 0; slave < ec_slavecount; ++slave)
  {
    EtherCAT_Msg_ptr msg;
    if (messages[slave].pop(msg))
    {
      memcpy(&Tx_Message[slave], msg.get(), sizeof(EtherCAT_Msg));
      isConfig[slave] = true;
    }
    EtherCAT_Msg* slave_dest = (EtherCAT_Msg*)(ec_slave[slave + 1].outputs);
    if (slave_dest)
      *(EtherCAT_Msg*)(ec_slave[slave + 1].outputs) = Tx_Message[slave];
  }
}

void runImpl()
{
  while (running)
  {
    EtherCAT_Run();
  }
}

void startRun()
{
  running = true;
  runThread = std::thread(runImpl);
}
