#include "control_task.hpp"

#include "cmsis_os.h"
#include "io/imu_task.hpp"
#include "pids.hpp"
float gravity_compensation;
float pitch_torque;

float a;
float b;
float c;
float d;

extern "C" void Control_Task()
{
  osDelay(100);
  can1.config();
  can1.start();
  can2.config();
  can2.start();
  yaw_motor.write_enable(can2.tx_data);
  can2.send(yaw_motor.tx_id);
  pitch_init();
  uint32_t previous_wake_time = osKernelSysTick();
  const uint32_t task_period_ms = 1;
  while (1) {
    global_mode_control();
    motor_enable();
    chassis_control();
    gimbal_control();
    fric_control();
    trigger_control();
    chassis_send();
    yaw_send();
    pitch_send();
    fric_send();
    trigger_send();
    osDelayUntil(&previous_wake_time, task_period_ms);
  }
}

void motor_enable()
{
  yaw_motor_erroeclear();
  if (yaw_motor.error == 0) {
    yaw_motor.write_enable(can2.tx_data);
    can2.send(yaw_motor.tx_id);
  }
  if (pitch_motor.error != 0) {
    pitch_init();
  }
  pitch_init();
}
void yaw_motor_erroeclear()
{
  if (yaw_motor.error != 1 && yaw_motor.error != 0) {
    yaw_motor.write_clear_error(can2.tx_data);
    can2.send(yaw_motor.tx_id);
    yaw_motor.write_enable(can2.tx_data);
    can2.send(yaw_motor.tx_id);
  }
}
void pitch_init()
{
  pitch_motor.cmd_set_single_parameter(run_mode, 0);  //设置为运控模式
  pitch_motor.write(can1.tx_data);
  can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.motor_id, pitch_motor.master_id);
  osDelay(1);
  pitch_motor.cmd_motor_enable();  //电机使能
  pitch_motor.write(can1.tx_data);
  can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.motor_id, pitch_motor.master_id);
}
void gimbal_control()
{
  if (Gimbal_Mode == GIMBAL_ZERO_FORCE) {
    yaw_motor.cmd(0.0f);
    pitch_motor.cmd(0.0f);
  }
  else if (Gimbal_Mode == GIMBAL_INITIAL) {
    gimbal_initial_control();
  }
  else if (Gimbal_Mode == GIMBAL_GYRO) {
    gimbal_gyro_control();
  }
  else if (Gimbal_Mode == GIMBAL_AUTO) {
#ifdef MPC
    if (!vis.control) {
      gimbal_gyro_control();
      return;
    }
    yaw_acc_pid.calc(yaw_angle.target, imu.yaw, vis.yaw_vel, imu.vyaw);
    auto yaw_motor_speed = imu.vyaw;
    auto yaw_torque = YAW_INERTIA * (vis.yaw_acc + yaw_acc_pid.out) +
                      YAW_DAMPING_COEFF * yaw_motor_speed +
                      (yaw_motor_speed > 0 ? 1.0f : -1.0f) * YAW_COULOMB_FORCE;
    yaw_spin_compensation_pid.calc(-v_chassis.w, 0.0f);
    if (yaw_torque > MAX_4310_TORQUE) {
      yaw_cmd_torque = sp::limit_max(yaw_torque, MAX_4310_TORQUE);
    }
    else {
      yaw_cmd_torque = sp::limit_max(yaw_torque + yaw_spin_compensation_pid.out, MAX_4310_TORQUE);
    }
    yaw_motor.cmd(yaw_cmd_torque);

    pitch_acc_pid.calc(pitch_angle.target, imu.pitch, vis.pitch_vel, imu.vpitch);
    pitch_torque = PITCH_INERTIA * (vis.pitch_acc + pitch_acc_pid.out) +
                   PITCH_DAMPING_COEFF * imu.vpitch +
                   (imu.vpitch > 0 ? 1.0f : -1.0f) * PITCH_COULOMB_FORCE -
                   std::cos(imu.pitch) * PITCH_GRAVITY_TORQUE;
    a = PITCH_INERTIA * (vis.pitch_acc + pitch_acc_pid.out);
    b = PITCH_DAMPING_COEFF * imu.vpitch;
    c = (imu.vpitch > 0 ? 1.0f : -1.0f) * PITCH_COULOMB_FORCE;
    d = -std::cos(imu.pitch) * PITCH_GRAVITY_TORQUE;
    pitch_motor.cmd(-pitch_torque);
#endif
  }
}
void gimbal_initial_control()
{
  yaw_pos_pid.calc(yaw_angle.target, yaw_angle.relative);
  yaw_speed_pid.calc(yaw_pos_pid.out, imu_vyaw_filter);
  yaw_motor.cmd(yaw_speed_pid.out);
  pitch_pos_pid.calc(pitch_angle.target, imu.pitch);
  pitch_speed_pid.calc(pitch_pos_pid.out, imu.vpitch);
  gravity_compensation = cos(imu.pitch) * TOR_PARAM;
  pitch_torque = -pitch_speed_pid.out + gravity_compensation;
  pitch_motor.cmd(pitch_torque);
}
void gimbal_gyro_control()
{  // 云台控制中我们采取imu传感器下的坐标系
  // 我们在云台初始化最终时已经将当时的target角设为了当时的imu角度，在那一角度的基础上进行加减计算新的目标角度
  yaw_pos_pid.calc(yaw_angle.target, imu.yaw);
  yaw_speed_pid.calc(yaw_pos_pid.out, imu_vyaw_filter);
  yaw_motor.cmd(yaw_speed_pid.out);
  pitch_pos_pid.calc(pitch_angle.target, imu.pitch);
  pitch_speed_pid.calc(pitch_pos_pid.out, imu.vpitch);
  gravity_compensation = cos(imu.pitch) * TOR_PARAM;
  pitch_torque = -pitch_speed_pid.out + gravity_compensation;
  pitch_motor.cmd(pitch_torque);
}
void chassis_control()
{
  if ((Chassis_Mode == CHASSIS_ZERO_FORCE) || (Chassis_Mode == CHASSIS_INITIAL)) {
    wheel_lb.cmd(0.0f);
    wheel_lf.cmd(0.0f);
    wheel_rb.cmd(0.0f);
    wheel_rf.cmd(0.0f);
  }
  else {
    wheel_give_torque =
      chassis_pid_cal(v_wheel_target.lf, v_wheel_target.lb, v_wheel_target.rf, v_wheel_target.rb);
    Pmax_get();
    chassis_power_control(&wheel_give_torque, &v_wheel, &v_wheel_target, infact_Pmax - 3.0f);
    // chassis_energy_control(wheel_speed, &wheel_give_torque);
    wheel_lf.cmd(wheel_give_torque.lf);
    wheel_lb.cmd(wheel_give_torque.lr);
    wheel_rf.cmd(wheel_give_torque.rf);
    wheel_rb.cmd(wheel_give_torque.rr);
  }
}

Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr)
{
  Wheel_Torque wheel_given_torque_temp;
  speed_lf_pid.calc(lf, wheel_lf.speed);
  speed_lr_pid.calc(lr, wheel_lb.speed);
  speed_rf_pid.calc(rf, wheel_rf.speed);
  speed_rr_pid.calc(rr, wheel_rb.speed);
  wheel_given_torque_temp.lf = speed_lf_pid.out;
  wheel_given_torque_temp.lr = speed_lr_pid.out;
  wheel_given_torque_temp.rf = speed_rf_pid.out;
  wheel_given_torque_temp.rr = speed_rr_pid.out;
  return wheel_given_torque_temp;
}

void fric_control()
{
  if (Fric_Mode == FRIC_DOWN) {
    fric_motor1.cmd(0.0f);
    fric_motor2.cmd(0.0f);
    fric_motor3.cmd(0.0f);
    fric_motor4.cmd(0.0f);
    fric_motor5.cmd(0.0f);
    fric_motor6.cmd(0.0f);
    fricmotor1_pid.data.iout = 0.0f;
    fricmotor2_pid.data.iout = 0.0f;
    fricmotor3_pid.data.iout = 0.0f;
    fricmotor4_pid.data.iout = 0.0f;
    fricmotor5_pid.data.iout = 0.0f;
    fricmotor6_pid.data.iout = 0.0f;
    return;
  }
  fricmotor1_pid.calc(-fric_1stSpeed_target, fric_motor1.speed);
  fricmotor2_pid.calc(fric_1stSpeed_target, fric_motor2.speed);
  fricmotor3_pid.calc(fric_1stSpeed_target, fric_motor3.speed);
  fricmotor4_pid.calc(fric_2ndSpeed_target, fric_motor4.speed);
  fricmotor5_pid.calc(-fric_2ndSpeed_target, fric_motor5.speed);
  fricmotor6_pid.calc(fric_2ndSpeed_target, fric_motor6.speed);

  fric_motor1.cmd(fricmotor1_pid.out);
  fric_motor2.cmd(fricmotor2_pid.out);
  fric_motor3.cmd(fricmotor3_pid.out);
  fric_motor4.cmd(fricmotor4_pid.out);
  fric_motor5.cmd(fricmotor5_pid.out);
  fric_motor6.cmd(fricmotor6_pid.out);
}
void trigger_control()
{
  if (Global_Mode == ZERO_FORCE || !pm02.robot_status.power_management_shooter_output) {
    trigger_motor.cmd(0.0f);
    return;
  }
  else {
    if (Trigger_Mode == TRIGGER_INITIAL) {
      trigger_initial_control();
    }
    else if (Fric_Mode == FRIC_ON && Trigger_Mode == SHOOT_READY) {
      shoot_control();
    }
    else {
      trigger_motor.cmd(0.0f);
    }
  }
}

void trigger_initial_control()
{
  triggerback_pos_pid1.calc(trigger_angle_target, trigger_motor.angle);
  //防超调之后回到原目标慢
  if (triggerback_pos_pid1.data.iout * (trigger_angle_target - trigger_motor.angle) < 0) {
    triggerback_pos_pid1.data.iout = 0.0f;
  }
  triggerback_speed_pid2.calc(triggerback_pos_pid1.out, trigger_motor.speed);
  trigger_motor.cmd(triggerback_speed_pid2.out);
}

void shoot_control()
{
  trigger_pos_pid1.calc(trigger_angle_target, trigger_motor.angle);
  trigger_speed_pid2.calc(trigger_pos_pid1.out, trigger_motor.speed);
  trigger_motor.cmd(trigger_speed_pid2.out);
  if (trigger_pos_pid1.out * (trigger_angle_target - trigger_motor.angle) < 0) {
    trigger_pos_pid1.data.iout = 0.0f;
  }
}