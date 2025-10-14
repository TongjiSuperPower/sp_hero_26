#include "controllers/control_task.hpp"

#include "A_HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controllers/chassis_controller/energy_control.hpp"
#include "controllers/chassis_controller/power_control.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "detect_task.hpp"
#include "io/imu_task.hpp"
#include "mode.hpp"
#include "pids.hpp"

uint8_t cap_send_num = 0;

//射击数据发送频率控制符
uint16_t shoot_mode_send_num = 0;
uint16_t q_send_num = 0;
//达妙使能帧控制符
uint16_t yaw_enable_num = 0;
uint16_t pitch_enable_num = 0;

void motor_enable();
void chassis_control();
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr);

extern "C" void Control_Task()
{
  osDelay(500);  // 等待各个任务初始化完成
  can2.config();
  can2.start();
  yaw_motor.write_enable(can2.tx_data);
  can2.send(yaw_motor.tx_id);
  while (1) {
    // 确定全局状态机
    global_mode_control();
    // 达妙失能检测,发送使能帧
    motor_enable();
    //底盘控制
    chassis_control();
    // can1 send一次；can2 send一次
    chassis_send();

    osDelay(1);
  }
}

//控制函数
//底盘轮组电机控制
void chassis_control()
{
  if (Chassis_Mode == CHASSIS_DOWN || Chassis_Mode == CHASSIS_INIT) {
    wheel_lf.cmd(0.0f);
    wheel_lr.cmd(0.0f);
    wheel_rf.cmd(0.0f);
    wheel_rr.cmd(0.0f);
    return;
  }
  wheel_give_torque = chassis_pid_cal(
    chassis_target_speed.lf, chassis_target_speed.lr, chassis_target_speed.rf,
    chassis_target_speed.rr);
  Pmax_get();
  chassis_power_control(
    &wheel_give_torque, &wheel_speed, &chassis_target_speed, infact_Pmax - 3.0f);
  chassis_energy_control(wheel_speed, &wheel_give_torque);
  wheel_lf.cmd(wheel_give_torque.lf);
  wheel_lr.cmd(wheel_give_torque.lr);
  wheel_rf.cmd(wheel_give_torque.rf);
  wheel_rr.cmd(wheel_give_torque.rr);
}

//底盘层面pid解算
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr)
{
  Wheel_Torque wheel_given_torque_temp;
  speed_lf_pid.calc(lf, wheel_lf_speed_filter);
  speed_lr_pid.calc(lr, wheel_lr_speed_filter);
  speed_rf_pid.calc(rf, wheel_rf_speed_filter);
  speed_rr_pid.calc(rr, wheel_rr_speed_filter);
  wheel_given_torque_temp.lf = speed_lf_pid.out;
  wheel_given_torque_temp.lr = speed_lr_pid.out;
  wheel_given_torque_temp.rf = speed_rf_pid.out;
  wheel_given_torque_temp.rr = speed_rr_pid.out;
  return wheel_given_torque_temp;
}

//达妙发送使能帧
void motor_enable(void)
{
  if (yaw_motor.error == 0) {
    yaw_motor.write_enable(can2.tx_data);
    can2.send(yaw_motor.tx_id);
  }
  // if (pitch_motor.error == 0) {
  //   pitch_motor.write_enable(can1.tx_data);
  //   can1.send(pitch_motor.tx_id);
  // }
  if (!yaw_motor_alive) {
    if (yaw_enable_num == 1000) {
      yaw_motor.write_enable(can2.tx_data);
      can2.send(yaw_motor.tx_id);
      yaw_enable_num = 0;
    }
    yaw_enable_num++;
  }
  // if (!pitch_motor_alive) {
  //   if (pitch_enable_num == 1000) {
  //     pitch_motor.write_enable(can1.tx_data);
  //     can1.send(pitch_motor.tx_id);
  //     pitch_enable_num = 0;
  //   }
  //   pitch_enable_num++;
  // }
}
