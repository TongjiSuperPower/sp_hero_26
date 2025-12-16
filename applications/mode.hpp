#ifndef MODE_HPP
#define MODE_HPP
#include "HERO_SELECTION.hpp"
#include "data/uart/uart_task.hpp"
typedef enum
{
  ZERO_FORCE,
  REMOTE,
  KEYBOARD,
} global_mode;
typedef enum
{
  CHASSIS_ZERO_FORCE,
  CHASSIS_INITIAL,
  CHASSIS_FOLLOW,
  CHASSIS_SPIN,
} chassis_mode;
typedef enum
{
  GIMBAL_ZERO_FORCE,
  GIMBAL_INITIAL,
  GIMBAL_GYRO,
  GIMBAL_AUTO,
  GIMBAL_LOB,
} gimbal_mode;
typedef enum
{
  FRIC_DOWN,
  FRIC_ON,
  FRIC_OFF,
} fric_mode;
typedef enum
{
  TRIGGER_DOWN,
  TRIGGER_INITIAL,
  SHOOT_READY
} trigger_mode;

#endif
void global_mode_control();
extern global_mode Global_Mode;
extern chassis_mode Chassis_Mode;
extern gimbal_mode Gimbal_Mode;
extern gimbal_mode Last_Gimbal_Mode;