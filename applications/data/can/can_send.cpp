#include "can_send.hpp"

#include "controllers/chassis_task/chassis_task.hpp"
#include "controllers/gimbal_task/gimbal_task.hpp"
#include "controllers/shoot_task/shoot_task.hpp"
void chassis_send()
{
  static uint8_t send_count = 0;
  send_count++;
  if (send_count >= 2) {
    wheel_lf.write(can2.tx_data);
    wheel_rf.write(can2.tx_data);
    wheel_lb.write(can2.tx_data);
    wheel_rb.write(can2.tx_data);
    can2.send(wheel_lf.tx_id);
    send_count = 0;
  }
}
void supercap_send()
{
  super_cap.write(
    can2.tx_data, pm02.robot_status.chassis_power_limit, pm02.power_heat.buffer_energy,
    pm02.robot_status.power_management_chassis_output);
  can2.send(super_cap.tx_id);
}
void yaw_send()
{
  yaw_motor.write(can2.tx_data);
  can2.send(yaw_motor.tx_id);
}
void pitch_send()
{
  pitch_motor.write(can1.tx_data);
  can1.send_ext(
    pitch_motor.communication_type, pitch_motor.tar_torque, pitch_motor.motor_id,
    pitch_motor.master_id);
}
void fric_send()
{
  fric_motor1.write(can1.tx_data);
  fric_motor2.write(can1.tx_data);
  fric_motor3.write(can1.tx_data);
  fric_motor4.write(can1.tx_data);
  can1.send(fric_motor1.tx_id);
  can1.tx_data[0] = 0;
  can1.tx_data[1] = 0;
  can1.tx_data[2] = 0;
  can1.tx_data[3] = 0;
  can1.tx_data[4] = 0;
  can1.tx_data[5] = 0;
  can1.tx_data[6] = 0;
  can1.tx_data[7] = 0;
  fric_motor5.write(can1.tx_data);
  fric_motor6.write(can1.tx_data);
  can1.send(fric_motor5.tx_id);
}
void trigger_send()
{
  trigger_motor.write(can2.tx_data);
  can2.send(trigger_motor.tx_id);
}