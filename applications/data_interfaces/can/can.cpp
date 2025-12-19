#include "can.hpp"

#include "controllers/chassis_controller/chassis_task.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"

//chassis CAN2（4个）
void chassis_send()
{
  static uint8_t count1 = 0;
  count1++;
  if (count1 >= 2) {
    wheel_lf.write(can2.tx_data);
    wheel_lr.write(can2.tx_data);
    wheel_rf.write(can2.tx_data);
    wheel_rr.write(can2.tx_data);
    can2.send(wheel_lf.tx_id);
    count1 = 0;
  }
}

//摩擦轮和拨弹轮 CAN1（3个）
void fric_send()
{
  fric_motor1.write(can1.tx_data);
  fric_motor2.write(can1.tx_data);
  fric_motor3.write(can1.tx_data);
  fric_motor4.write(can1.tx_data);
  can1.send(fric_motor1.tx_id);
}

void trigger_send()
{
  trigger_motor.write(can2.tx_data);
  can2.send(trigger_motor.tx_id);
}

//yaw CAN2（1个）
void yaw_send()
{
  yaw_motor.write(can2.tx_data);
  can2.send(yaw_motor.tx_id);
}

//pitch CAN1（1个）
void pitch_send()
{
  pitch_motor.write(can1.tx_data);
  can1.send_ext(
    pitch_motor.communication_type, pitch_motor.tar_torque, pitch_motor.motor_id,
    pitch_motor.master_id);
}

void super_cap_send()
{
  static uint8_t count = 0;
  count++;
  if (count >= 10) {
    super_cap.write(
      can2.tx_data, pm02.robot_status.chassis_power_limit, pm02.power_heat.buffer_energy,
      pm02.robot_status.power_management_chassis_output);
    can2.send(super_cap.tx_id);
    count = 0;
  }
}