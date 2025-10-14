#ifndef SHOOT_TASK_HPP
#define SHOOT_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "io/vision/vision.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/pid/pid.hpp"
// -------------------- 控制参数 --------------------
constexpr float T_SHOOT = 1e-3f;  // 控制周期, 单位: s
//摩擦轮转速rad/s
#ifdef HERO_DOG
constexpr float FRIC_SPEED = 390.0f; 
#endif

#ifdef HERO_THREE_WHEELS
constexpr float FRIC_SPEED = 670.0f;  //23.5度 21.5射速
#endif

// -------------------- SHOOT_INIT相关 --------------------
//拨弹轮堵转反转角度
constexpr float TRIGGER_BACK_ANGLE = 0.6f;  //rad
//拨弹轮双发反转角度
constexpr float TRIGGER_SINGLE_BACK_ANGLE = 0.4f;  //rad

// -------------------- SHOOT_READY_SINGLE相关 --------------------
//每次射击冷却时间
constexpr uint16_t SHOOT_COLD_TIME = 500;  //ms
//每次射击拨弹轮旋转角度
constexpr float SHOOT_ANGLE_ADD = 1.0472f;  //rad  3.1415926*2/6=1.0472
//双发判断一次射击摩擦轮降速阈值
#ifdef HERO_DOG
constexpr float FRIC_SPEED_REDUCE = 17.5f;
#endif
#ifdef HERO_THREE_WHEELS
constexpr float FRIC_SPEED_REDUCE = 17.0f;
#endif
//双发检测持续时间
constexpr uint16_t DETECT_TIME = 500;
constexpr float HEAT_PER_SHOT = 40.0f;

// -------------------- 对外硬件 --------------------
inline sp::RM_Motor trigger_motor(5, sp::RM_Motors::M3508, RADUCTION_RATIO);
inline sp::RM_Motor fric_motor1(1, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor2(2, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor3(3, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor4(4, sp::RM_Motors::M3508, 1);

// -------------------- 对外接口 --------------------
extern uint8_t shoot_mode_flag;
//发送给拨弹轮的扭矩（用于判断拨弹轮是否堵转）
extern float trigger_give_torque;

void shoot_mode_init(void);
void fric_mode_control(void);
void trigger_mode_control(void);
void fric_cmd(void);
void trigger_cmd(void);
void shoot_init_cmd(void);
void shoot_single_permission(void);
void shoot_double_detect();
#endif