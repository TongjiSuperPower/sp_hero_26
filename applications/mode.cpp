#include "mode.hpp"
global_mode Global_Mode = ZERO_FORCE;
global_mode Last_Global_Mode = ZERO_FORCE;
chassis_mode Chassis_Mode;
gimbal_mode Gimbal_Mode;
gimbal_mode Last_Gimbal_Mode;

void global_mode_control()
{
#ifdef DT7
  Last_Global_Mode = Global_Mode;
  if (
    remote.sw_r == sp::DBusSwitchMode::DOWN || !pm02.robot_status.power_management_gimbal_output) {
    Global_Mode = ZERO_FORCE;
    return;
  }
  if (remote.sw_r == sp::DBusSwitchMode::MID) Global_Mode = KEYBOARD;
  if (remote.sw_r == sp::DBusSwitchMode::UP) Global_Mode = REMOTE;
#endif
}