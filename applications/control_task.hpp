#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP
#include "HERO_SELECTION.hpp"
#include "controllers/chassis_task/chassis_task.hpp"
#include "controllers/gimbal_task/gimbal_task.hpp"
#include "controllers/power_limit/power_limit.hpp"
#include "controllers/shoot_task/shoot_task.hpp"
#include "data/can/can_receive.hpp"
#include "data/can/can_send.hpp"
#include "io/can/can.hpp"
#include "io/vision/vision.hpp"
#include "mode.hpp"

#endif
// -------------------- 控制参数 --------------------
//计算力矩控制参数
#ifdef HERO_DOG
// m = 0.1001  b = 0.1862  c = 0.3601
// m = 0.1015, b = 0.2016, c = 0.2792
//m = 0.1031, b = 0.2252, c = 0.2766
constexpr float YAW_INERTIA = 0.1031f;
constexpr float YAW_DAMPING_COEFF = 0.2252f;
constexpr float YAW_COULOMB_FORCE = 0.2766f;

// m = 0.0331, b = 0.2112, c = 0.1037, d = 1.8725
// m = 0.0349, b = 0.2383, c = 0.0796, d = 1.8778
// m = 0.0395  b = 0.2486  c = 0.0800  d = 2.0584
//m = 0.0470, b = 0.2353, c = 0.0264, d = 1.9543
//m = 0.0468, b = 0.2527, c = 0.0107, d = 1.9504
constexpr float PITCH_INERTIA = 0.0400;
constexpr float PITCH_DAMPING_COEFF = 0.2353;
constexpr float PITCH_COULOMB_FORCE = 0.0800;
constexpr float PITCH_GRAVITY_TORQUE = 1.9543;
#endif
//重力补偿
// 云台平衡力矩系数N.M
#ifdef HERO_DOG
constexpr float TOR_PARAM = 1.5f;
#endif
Wheel_Torque chassis_pid_cal(float lf, float lb, float rf, float rb);
void motor_enable();
void chassis_control();
void gimbal_control();
void gimbal_initial_control();
void gimbal_gyro_control();
void yaw_motor_erroeclear();
void pitch_init();
void fric_control();
void trigger_control();
void trigger_initial_control();
void shoot_control();