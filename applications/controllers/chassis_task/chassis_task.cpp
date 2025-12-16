#include "chassis_task.hpp"

#include "controllers/gimbal_task/gimbal_task.hpp"
#include "data/uart/uart_task.hpp"
#include "detect_task.hpp"
#include "pids.hpp"

bool chassis_InitialFlag = false;
uint8_t chassis_initialoverTime = 0;
uint8_t chassis_init_time = 0;
//低压放电保护 0：电压低不可使用电容  1：可使用电容
uint8_t low_vol_flag = 1;
float infact_Pmax = 0.0f;
//总使用能量
float energy_sum = 0.0f;
//底盘速度的期望值
chassis_speed v_chassis = {0.0f, 0.0f, 0.0f};
chassis_speed v_chassis_spincorrect = {0.0f, 0.0f, 0.0f};
//当前底盘速度解算到各个轮子之上
wheel_speed v_wheel = {0.0f, 0.0f, 0.0f, 0.0f};
//速度赋值完后目标速度结算到各个轮子之上
wheel_speed v_wheel_target = {0.0f, 0.0f, 0.0f, 0.0f};
//pid算出来的预期扭矩
Wheel_Torque wheel_give_torque = {0.0f, 0.0f, 0.0f, 0.0f};
bool spin_revert_flag;

extern "C" void Chassis_Task()
{
  while (1) {
    //总能量更新
    if (game_start_flag) {
      energy_sum += super_cap.power_in * 0.001f;
    }
    else {
      energy_sum = 0.0f;
    }
    chassis_mode_control();
    if (chassis_InitialFlag) {
      if (chassis_alive) {
        chassis_initialoverTime++;
      }
      chassis_init_time++;
      if (chassis_initialoverTime == 100 || chassis_init_time == 7000) {
        chassis_initialoverTime = 0;
        chassis_init_time = 0;
        chassis_InitialFlag = false;
      }
    }
    chassis_calculate();
    chassis_gimbal_cooperate(&v_chassis, yaw_angle.relative);
    wheel_speed_calculate(&v_chassis, &v_wheel_target);
    osDelay(1);
  }
}
void chassis_mode_control()
{
  static bool readytoInitial = pm02.robot_status.power_management_chassis_output;
  if (!readytoInitial && pm02.robot_status.power_management_chassis_output) {
    chassis_InitialFlag = true;
  }
  readytoInitial = pm02.robot_status.power_management_chassis_output;
  if (chassis_InitialFlag) {
    Chassis_Mode = CHASSIS_INITIAL;
    return;
  }
  if (Global_Mode == ZERO_FORCE || Gimbal_Mode == GIMBAL_INITIAL || Gimbal_Mode == GIMBAL_LOB) {
    Chassis_Mode = CHASSIS_ZERO_FORCE;
  }
  else {
#ifdef DT7
    //DT7遥控器模式
    if (Global_Mode == REMOTE) {
      if (remote.ch_lu < 0) {
        Chassis_Mode = CHASSIS_SPIN;
        spin_revert_flag = true;
      }
      else if (remote.ch_lu > 0) {
        Chassis_Mode = CHASSIS_SPIN;
        spin_revert_flag = false;
      }
      else {
        Chassis_Mode = CHASSIS_FOLLOW;
      }
    }

    //键鼠模式
    if (Global_Mode == KEYBOARD) {
      //不在小陀螺模式时，底盘进入跟随模式
      if (!key_spin) {
        Chassis_Mode = CHASSIS_FOLLOW;
      }
      //SHIFT放在后面保证任何模式下按下shift都进入小陀螺
      if (key_spin) {
        Chassis_Mode = CHASSIS_SPIN;
      }
    }
#endif
  }
}

void chassis_calculate()
{
  v_wheel.lf = wheel_lf.speed;
  v_wheel.lb = wheel_lb.speed;
  v_wheel.rf = wheel_rf.speed;
  v_wheel.rb = wheel_rb.speed;
  switch (Chassis_Mode) {
    case CHASSIS_ZERO_FORCE: {
      v_chassis.vx = 0.0f;
      v_chassis.vy = 0.0f;
      v_chassis.w = 0.0f;
      break;
    }
    case CHASSIS_INITIAL: {
      if (!chassis_InitialFlag) {
        v_chassis.vx = 0.0f;
        v_chassis.vy = 0.0f;
        v_chassis.w = 0.0f;
      }
      break;
    }
    case CHASSIS_FOLLOW: {
      chassis_followcal();
      break;
    }
    case CHASSIS_SPIN: {
      chassis_spincal();
      break;
    }
    default:
      break;
  }
}
void chassis_followcal()
{
  if (Global_Mode == REMOTE) {
    v_chassis.vx = remote.ch_lv * REMOTE_V;
    v_chassis.vy = -remote.ch_lh * REMOTE_V;
    ChassisFollow_wz_pid.calc(0.0f, yaw_angle.relative);
    v_chassis.w = -ChassisFollow_wz_pid.out;
  }
  else if (Global_Mode == KEYBOARD) {
    float vx = 0.0f;
    float vy = 0.0f;
    if (key_move_x_up) vx = KEYBOARD_V;
    if (key_move_x_down) vx = -KEYBOARD_V;
    if (key_move_y_up) vy = KEYBOARD_V;
    if (key_move_y_down) vy = -KEYBOARD_V;
    v_chassis.vx = vx;
    v_chassis.vy = vy;
    ChassisFollow_wz_pid.calc(0.0f, yaw_angle.relative);
    v_chassis.w = -ChassisFollow_wz_pid.out;
  }
}
//将底盘速度变换到云台坐标系下，防止出现在底盘跟随云台过程中，底盘仍按照自己的坐标系运动
void chassis_gimbal_cooperate(chassis_speed * speed_given, float yaw_angle_relative)
{
  float vx = speed_given->vx;
  float vy = speed_given->vy;
  speed_given->vx = vx * cosf(yaw_angle.relative) -vy * sinf(yaw_angle.relative);
  speed_given->vy = vx * sinf(yaw_angle.relative) + vy * cosf(yaw_angle.relative);
 
}
void chassis_spincal()
{
  if (Global_Mode == REMOTE) {
    if (spin_revert_flag) {
      v_chassis.vx = -remote.ch_lv * REMOTE_V;
      v_chassis.vy = -remote.ch_lh * REMOTE_V;
      ChassisFollow_wz_pid.calc(0.0f, yaw_angle.relative);
      v_chassis.w = -ChassisFollow_wz_pid.out + SPIN_W;
    }
    if (!spin_revert_flag) {
      v_chassis.vx = -remote.ch_lv * REMOTE_V;
      v_chassis.vy = -remote.ch_lh * REMOTE_V;
      ChassisFollow_wz_pid.calc(0.0f, yaw_angle.relative);
      v_chassis.w = -ChassisFollow_wz_pid.out - SPIN_W;
    }
  }
  if (Global_Mode == KEYBOARD) {
    float vx_spin = 0.0f;
    float vy_spin = 0.0f;
    v_chassis.w=SPIN_W;

    if (infact_Pmax <= 150.0f) {
      // vx:前后  vy：左右
      vx_spin = (key_move_x_up ? SPIN_V * v_chassis.w / SPIN_W : 0.0f) +
                (key_move_x_down ? -SPIN_V * v_chassis.w / SPIN_W : 0.0f) +
                (key_move_y_up ? 0.7f * SPIN_V * v_chassis.w / SPIN_W : 0.0f) +
                (key_move_y_down ? -0.7f * SPIN_V * v_chassis.w / SPIN_W : 0.0f);
      vy_spin = (key_move_x_up ? -0.7f * SPIN_V * v_chassis.w / SPIN_W : 0.0f) +
                (key_move_x_down ? 0.7f * SPIN_V * v_chassis.w / SPIN_W : 0.0f) +
                (key_move_y_up ? SPIN_V * v_chassis.w / SPIN_W : 0.0f) +
                (key_move_y_down ? -SPIN_V * v_chassis.w / SPIN_W : 0.0f);
      vx_spin = sp::limit_max(vx_spin, SPIN_V);
      vy_spin = sp::limit_max(vy_spin, SPIN_V);
    }
    else {
      vx_spin = (key_move_x_up ? 1.3f : 0.0f) + (key_move_x_down ? -1.3f : 0.0f);
      vy_spin = (key_move_y_up ? 1.3f : 0.0f) + (key_move_y_down ? -1.3f : 0.0f);
    }

    v_chassis.vx = vx_spin;
    v_chassis.vy = vy_spin;
  }
}

void wheel_speed_calculate(chassis_speed * speed_given, wheel_speed * wheel_speed_out)
{
  mecanum.calc(speed_given->vx, speed_given->vy, speed_given->w);
  wheel_speed_out->lf = mecanum.speed_lf;
  wheel_speed_out->lb = mecanum.speed_lr;
  wheel_speed_out->rf = mecanum.speed_rf;
  wheel_speed_out->rb = mecanum.speed_rr;
}
void Pmax_get()
{
#ifdef RMUC
  //电容低电保护
  if (low_vol_flag == 0 && super_cap.voltage >= 11.2) {
    low_vol_flag = 1;
  }
  if (low_vol_flag == 1 && super_cap.voltage < 6.2) {
    low_vol_flag = 0;
  }
  //剩余能量大于30%时 01111B = 16D
  if (pm02.buff.remaining_energy > 16) {
    if (!key_cap) {
      infact_Pmax = pm02.robot_status.chassis_power_limit;
    }
    else {
      //根据电容的电压大小增加额外功率
      if (super_cap.voltage >= 6 && low_vol_flag == 1) {
        infact_Pmax = std::min(
          CAP_MAX_POWER, (super_cap.voltage - 6.0f) * 20 + pm02.robot_status.chassis_power_limit);
      }
      else {
        infact_Pmax = pm02.robot_status.chassis_power_limit;
      }
    }
  }
  else {
    //剩余能量不足时限制最大功率为80W
    if (!key_cap) {
      if (pm02.robot_status.chassis_power_limit > 80.0f) {
        infact_Pmax = 80.0f;
      }
      else {
        infact_Pmax = pm02.robot_status.chassis_power_limit;
      }
    }
    else {
      //根据电容的电压大小增加额外功率
      if (super_cap.voltage >= 6 && low_vol_flag == 1) {
        infact_Pmax = std::min(
          CAP_MAX_POWER, (super_cap.voltage - 6.0f) * 20 + pm02.robot_status.chassis_power_limit);
      }
      else {
        infact_Pmax = pm02.robot_status.chassis_power_limit;
      }
    }
  }
#endif
  buff_energy_p_limited();
}

void buff_energy_p_limited() {}
// {
//   if (pm02.power_heat.buffer_energy < 20.0f) {
//     //infact_Pmax 确定原则：x<45/3(对抗赛的一级功率/3：虚弱时最小功率限制)；x-3(后面留的余量)>K3，防止功率控制无解
//     infact_Pmax = K3 + 4.0f;
//   }
// }