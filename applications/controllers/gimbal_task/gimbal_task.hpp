#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP
#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "io/imu_task.hpp"
#include "keys.hpp"
#include "mode.hpp"
#include "motor/cybergear_motor/cybergear_motor.hpp"
#include "motor/dm_motor/dm_motor.hpp"
// NOTE: keep header guard until file end
//控制参数
constexpr float T_GIMBAL = 1e-3;
typedef struct
{
  float installed;
  float relative;
  float add;
  float target;
} Yaw_angle;
typedef struct
{
  float installed;
  float relative;
  float add;
  float target;
} Pitch_angle;

#ifdef HERO_DOG
//遥控器模式云台数据
constexpr float W_MAX = 0.004f;  //rad/ms
//键鼠模式鼠标系数数据
constexpr float MOUSE_DPI = 0.866f;  //联盟赛0.00006f
//调头冷却时间
constexpr float TURNOVER_COLDTIME = 800;  //ms

constexpr float IMU_PITCH_ANGLE_MAX = 0.33f;   //Pitch轴限位    最大角度0.50
constexpr float IMU_PITCH_ANGLE_MIN = -0.58f;  //Pitch轴限位    最小角度-0.25
#endif
void gimbal_mode_control();
void gimbal_calculate();
void gimbal_paramInitialize();
void gimbal_zeroforce();
void gimbal_initial();
void gimbal_gyro();
void gimbal_auto();
void gimbal_lob();

inline sp::DM_Motor yaw_motor(0x08, 0x04, 3.141593f, 30.0f, 10.0f);
inline sp::CyberGear_Motor pitch_motor(
  Master_CAN_ID, CyberGear_CAN_ID, CYBERGEAR_MAX_POSITION, CYBERGEAR_MAX_SPEED,
  CYBERGEAR_MAX_TORQUE);

extern Yaw_angle yaw_angle;
extern Pitch_angle pitch_angle;
extern float yaw_cmd_torque;

#endif  // GIMBAL_TASK_HPP
