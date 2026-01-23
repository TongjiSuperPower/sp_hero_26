#include "cmsis_os.h"
#include "controllers/control_task.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "io/imu_task.hpp"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart1);

extern "C" void Plotter_Task()
{
  /* USER CODE BEGIN Plotter_Task */
  /* Infinite loop */
  while (1) {
    plotter.plot(
      // trigger_motor.angle
      // imu.pitch,pitch_torque,1
      // imu.pitch,pitch_target_angle, pitch_torque,a,b,c,d
      // trigger_motor.angle,trigger_target_angle
      // -fric_motor1.speed, vis.fire, pitch_target_angle, imu.pitch, yaw_target_angle, imu.yaw
      // imu.pitch,pitch_target_angle, pitch_torque, gravity_compensation,pitch_encode_speed_pid.out,pitch_encode_pos_pid.out
      // pitch_target_angle,pitch_relative_angle,pitch_encode_pos_pid.out,pitch_encode_pos_pid.data.pout,pitch_encode_pos_pid.data.iout,pitch_encode_pos_pid.data.dout
      // trigger_target_angle, trigger_motor.angle
      // super_cap.power_in - super_cap.power_out
      shoot_count, cal_heat, -fric_motor1.speed,pm02.power_heat.shooter_42mm_barrel_heat);
    // plotter.plot(
    //   imu.yaw, yaw_target_angle, yaw_acc_pid.out, yaw_acc_pid.data.pout, yaw_acc_pid.data.iout,
    // yaw_acc_pid.data.dout, yaw_cmd_torque, yaw_motor.torque, vis.yaw_acc, bmi088.acc[1], vis.yaw_vel, imu.vyaw);

    osDelay(10);
  }
  /* USER CODE END Plotter_Task */
}