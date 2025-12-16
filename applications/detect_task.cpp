#include "detect_task.hpp"

#include "cmsis_os.h"
#include "controllers/chassis_task/chassis_task.hpp"
#include "controllers/gimbal_task/gimbal_task.hpp"
#include "controllers/shoot_task/shoot_task.hpp"
#include "keys.hpp"
bool chassis_alive = false;
bool gimbal_alive = false;
bool shoot_alive = false;
bool yaw_motor_alive = false;
bool trigger_motor_alive = false;
bool pitch_motor_alive = false;
bool fric_motor_alive = false;
uint8_t reset_count = 50;
//打弹颗数计数
uint32_t shoot_count = 0;
bool count_flag = false;

extern "C" void Detect_Task()
{
  while (1) {
#ifdef DT7
    Cboard_reset(remote.keys.g);
#endif
    motor_detect();
    osDelay(10);
    // 检测底盘是否存活
  }
}
void Cboard_reset(bool key1)
{
  if (key1 && reset_count > 0) {
    reset_count--;
  }
  else {
    reset_count = 50;
  }
  if (reset_count == 0) {
    HAL_NVIC_SystemReset();
  }
}
void motor_detect()
{
  auto stamp_ms = osKernelSysTick();  // 获取当前的系统时间戳（以毫秒为单位）
  yaw_motor_alive = yaw_motor.is_alive(stamp_ms);
  //   trigger_motor_alive = trigger_motor.is_alive(stamp_ms);
  pitch_motor_alive = pitch_motor.is_alive(stamp_ms);
  if (pm02.robot_status.power_management_chassis_output) {
    chassis_alive = wheel_lf.is_alive(stamp_ms) && wheel_lb.is_alive(stamp_ms) &&
                    wheel_rf.is_alive(stamp_ms) && wheel_rb.is_alive(stamp_ms);
  }
  else {
    chassis_alive = true;
  }

  if (pm02.robot_status.power_management_gimbal_output) {
    gimbal_alive = yaw_motor.is_alive(stamp_ms) && pitch_motor.is_alive(stamp_ms);
  }
  else {
    gimbal_alive = true;
  }
}
void shoot_count_detect()
{
  last_fric_speed = fric_speed;
  fric_speed = (-fric_motor1.speed + fric_motor2.speed) / 2.0f;
  if (Fric_Mode == FRIC_ON) {
    if (last_fric_speed - fric_speed > 20.0f && count_flag == false) {
      shoot_count++;
      count_flag = true;
    }
    else {
      count_flag = false;
    }
  }
}