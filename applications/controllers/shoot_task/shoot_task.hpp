#ifndef SHOOT_TASK_HPP
#define SHOOT_TASK_HPP
#include "HERO_SELECTION.hpp"
#include "keys.hpp"
#include "mode.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#ifdef HERO_DOG
constexpr float FRIC_SPEED_FIRST = 320.0f;
constexpr float FRIC_SPEED_SECOND = 600.0f;
constexpr float FRIC_SPEED_FIRST_LOB = 100.0f;
constexpr float FRIC_SPEED_SECOND_LOB = 550.0F;
constexpr float HEAT_PRE_SHOT = 100.0f;
constexpr uint16_t SHOOT_COLD_TIME = 500;
constexpr float SHOOT_ANGLE_ADD = 1.0472f;  //rad  3.1415926*2/6=1.0472
constexpr float TRIGGER_INIT_ANGLE = 0.0f;  //rad
#endif
//状态机变量
extern fric_mode Fric_Mode;
extern fric_mode Last_Fric_Mode;
extern trigger_mode Trigger_Mode;
//电机转速变量
extern float fric_1stSpeed_target;
extern float fric_2ndSpeed_target;
extern bool fric_speedStablized_flag;
extern uint16_t fric_targetchangeCount;
extern bool shoot_initialFlag;
extern float trigger_speed_target;
extern float trigger_angle_target;
extern bool count_flag;

//弹速相关变量
extern float initial_speed;
extern float last_initial_speed;
//实例化电机
inline sp::RM_Motor fric_motor1(1, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor2(2, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor3(3, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor4(4, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor5(5, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor6(6, sp::RM_Motors::M3508, 1);
inline sp::DM_Motor trigger_motor(0x01, 0x00, 3.1415926f, 30.0f, 10.0f);
//控制函数
void fric_mode_control();
void fric_calculate();
void trigger_mode_control();
void trigger_calculate();
void shoot_paramInitial();
void trigger_initialcal();
void shoot_permission();
void shoot_mode_init();
float trigger_near_work_position();
#endif  // SHOOT_TASK_HPP
