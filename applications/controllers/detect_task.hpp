#ifndef _DETECT_TASK_HPP_
#define _DETECT_TASK_HPP_
#include <stdint.h>

// -------------------- 对外接口 --------------------
//拨弹轮堵转检测
extern bool trigger_block_flag;
extern uint16_t trigger_num;
//电机掉线检测
extern bool motor_alive;
extern bool yaw_motor_alive;
extern bool pitch_motor_alive;
extern bool fric_motor_alive;
extern bool chassis_alive;
extern bool gimbal_alive;
extern bool shoot_alive;
//坡度检测
extern float slope_angle;
extern uint32_t slope_time_10;
extern uint32_t slope_time_20;
//电池电压检测
extern bool low_battery_flag;
extern float voltage;
//被墙卡住检测
extern bool stuck_flag;

#endif