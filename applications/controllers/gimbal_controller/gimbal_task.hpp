#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "controllers/pids.hpp"
#include "io/servo/servo.hpp"
#include "io/vision/vision.hpp"
#include "motor/cybergear_motor/cybergear_motor.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "tools/yaw_feedward/yaw_feedward.hpp"

// -------------------- 控制参数 --------------------
constexpr float T_GIMBAL = 1e-3;

#ifdef HERO_DOG
//遥控器模式云台数据
constexpr float W_MAX = 0.004f;  //rad/ms

//pitch向上为负
constexpr float IMU_PITCH_ANGLE_MAX = 0.25f;   //Pitch轴限位    最大角度0.50
constexpr float IMU_PITCH_ANGLE_MIN = -0.50f;  //Pitch轴限位    最小角度-0.25
#endif

#ifdef HERO_SIX_WHEELS
//遥控器模式云台数据
constexpr float W_MAX = 0.004f;  //rad/ms

constexpr float IMU_PITCH_ANGLE_MAX = 0.35f;   //Pitch轴限位    最大角度0.54
constexpr float IMU_PITCH_ANGLE_MIN = -0.50f;  //Pitch轴限位    最小角度-0.49
#endif

//键鼠模式鼠标系数数据
constexpr float MOUSE_DPI = 0.866f;  //联盟赛0.00006f
//调头冷却时间
constexpr float TURNOVER_COLDTIME = 800;  //ms

//重力补偿
// 云台平衡力矩系数N.M
#ifdef HERO_DOG
constexpr float TOR_PARAM = 1.5f;
// 重心偏角
constexpr float OFFSET_ANGLE = 0.0f;  // rad
#endif
#ifdef HERO_SIX_WHEELS
constexpr float TOR_PARAM = 1.4517f;
//重心偏角
constexpr float OFFSET_ANGLE = 0.0f;  // rad
#endif

// -------------------- 对外硬件 --------------------
inline sp::DM_Motor yaw_motor(0x08, 0x04, 3.141593f, 30.0f, 10.0f);
inline sp::CyberGear_Motor pitch_motor(
  0x06, 0x00, CYBERGEAR_MAX_POSITION, CYBERGEAR_MAX_SPEED,
  CYBERGEAR_MAX_TORQUE);
inline sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 270.0f);  // 开发板最上面的PWM端口, 270度舵机

// -------------------- 对外调试 --------------------
extern float yaw_offecd_ecd_angle;
extern float yaw_target_angle;
extern float pitch_target_angle;
extern float yaw_relative_angle;
extern float pitch_relative_angle;
extern float yaw_cmd_torque;
extern float pitch_torque;
extern float gravity_compensation;
extern float servo_position;
extern bool last_key_lob_mode;
extern bool last_key_autoaim;
extern uint16_t gimbal_init_time;
extern uint16_t gimbal_init_over_time;

#endif