#ifndef CHASSIS_TASK_HPP
#define CHASSIS_TASK_HPP

#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "keys.hpp"
#include "mode.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "tools/mecanum/mecanum.hpp"

// -------------------- 类型定义 --------------------
struct chassis_speed
{
  float vx;  // 前进速度, 单位: m/s
  float vy;  // 左移速度, 单位: m/s
  float w;   // 逆时针角速度, 单位: rad/s
};

struct wheel_speed
{
  float lf;  // 左前轮速度, 单位: rad/s
  float lb;  // 左后轮速度, 单位: rad/s
  float rf;  // 右前轮速度, 单位: rad/s
  float rb;  // 右后轮速度, 单位: rad/s
};

struct Wheel_Torque
{
  float lf;
  float lr;
  float rf;
  float rr;
};

// -------------------- API 声明 --------------------
void chassis_mode_control();
void chassis_reversecal();
void chassis_calculate();
void chassis_followcal();
void chassis_spincal();
void Pmax_get();
void buff_energy_p_limited();
void chassis_gimbal_cooperate(chassis_speed * chassis_speed_in, float gimbal_yaw_angle_relative);
void wheel_speed_calculate(chassis_speed * speed_given, wheel_speed * wheel_speed_out);

// -------------------- 控制参数 --------------------
constexpr float REMOTE_V = 3.0f;    // m/s
constexpr float KEYBOARD_V = 3.2f;  // m/s
constexpr float SPIN_W = 7.3f;      // rad/s
constexpr float SPIN_V = 1.3f;      // m/s

// -------------------- 机械参数 --------------------
#ifdef HERO_DOG

constexpr float WHEEL_RADIUS = 77e-3f;       // m
constexpr float CHASSIS_LENGTH = 396.2e-3f;  // m
constexpr float CHASSIS_WIDTH = 356.47e-3f;  // m

#endif
//超级电容
inline sp::SuperCap super_cap(sp::SuperCapMode::AUTOMODE);
extern uint8_t low_vol_flag;
constexpr float CAP_MAX_POWER = 400.0f;  //16V时电容最大放电功率
//电容最大放电功率=25*电压

#ifdef HERO_DOG
inline sp::RM_Motor wheel_lf(1, sp::RM_Motors::M3508, RADUCTION_RATIO);
inline sp::RM_Motor wheel_lb(4, sp::RM_Motors::M3508, RADUCTION_RATIO);
inline sp::RM_Motor wheel_rf(2, sp::RM_Motors::M3508, RADUCTION_RATIO);
inline sp::RM_Motor wheel_rb(3, sp::RM_Motors::M3508, RADUCTION_RATIO);
inline sp::Mecanum mecanum((0.5 * WHEEL_RADIUS), (0.5 * CHASSIS_LENGTH), (0.5 * CHASSIS_WIDTH));
#endif

extern chassis_speed v_chassis;
extern wheel_speed v_wheel;
extern wheel_speed v_wheel_target;
extern Wheel_Torque wheel_give_torque;

#endif  // CHASSIS_TASK_HPP