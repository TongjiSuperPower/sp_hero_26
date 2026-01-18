#include "gimbal_task.hpp"

#include "data/uart/uart_task.hpp"
#include "tools/math_tools/math_tools.hpp"
Yaw_angle yaw_angle = {0.0f, 0.0f, 0.0f, 0.0f};
Pitch_angle pitch_angle{0.0f, 0.0f, 0.0f, 0.0f};
uint16_t gimbal_initial_over_time = 0;
//掉头冷却时间1s
uint16_t turnover_cold_time = TURNOVER_COLDTIME;
//发送给上位机的射击状态标识符
uint8_t shoot_mode_flag = 0;
//云台初始化标识符
bool gimbal_InitialFlag = 0;
// 上一次 lob 键状态（用于上升沿检测）
static bool last_key_lob_mode_local = false;
//输入给yaw的扭矩
float yaw_cmd_torque = 0.0f;
//初始化条件：每次上电后回中，情况1：初次上电之后，情况2：断电重启之后
extern "C" void Gimbal_Task()
{
  osDelay(200);
  gimbal_paramInitialize();
  while (1) {
    gimbal_mode_control();
    gimbal_calculate();
    osDelay(1);
  }
}
void gimbal_paramInitialize()
{
#ifdef HERO_DOG
  yaw_angle.installed = 1.2625838f;
  pitch_angle.installed = 0.37f;
#endif
}
void gimbal_mode_control()
{
  if (Global_Mode == ZERO_FORCE) {
    Gimbal_Mode = GIMBAL_ZERO_FORCE;
    Last_Gimbal_Mode = GIMBAL_ZERO_FORCE;
    shoot_mode_flag = 0;
  }

  if (gimbal_InitialFlag == 1) return;

  if (Global_Mode == REMOTE) {
    Gimbal_Mode = GIMBAL_GYRO;
    shoot_mode_flag = 0;
  }
  if (Global_Mode == KEYBOARD) {
    // 定义一个静态变量来记录“基础模式”（是普通还是吊射）
    static gimbal_mode Keyboard_Base_Mode = GIMBAL_GYRO;

    // --- LOB 切换逻辑 ---
    bool cur_lob = key_lob_mode;
    bool lob_edge = (cur_lob && !last_key_lob_mode_local);

    if (lob_edge) {
      // 如果当前基础模式是吊射，就切回普通；反之切成吊射
      if (Keyboard_Base_Mode == GIMBAL_LOB) {
        Keyboard_Base_Mode = GIMBAL_GYRO;
      }
      else {
        Keyboard_Base_Mode = GIMBAL_LOB;
      }
    }
    last_key_lob_mode_local = cur_lob;

    if (key_autoaim) {
      // 优先级最高：自瞄
      // 这里不更新 Base_Mode，只临时覆盖 Gimbal_Mode
      Gimbal_Mode = GIMBAL_AUTO;
      shoot_mode_flag = 1;
    }
    else {
      // 没有自瞄时，恢复到基础模式（可能是 GYRO 也可能是 LOB）
      Gimbal_Mode = Keyboard_Base_Mode;

      // 注意：如果是 LOB 模式，通常不应该置 0
      if (Gimbal_Mode == GIMBAL_LOB) {
        // 吊射时 shoot_mode_flag 保持不变或特定值
      }
      else {
        shoot_mode_flag = 0;
      }
    }

    if (Gimbal_Mode != Last_Gimbal_Mode) {
      Last_Gimbal_Mode = Gimbal_Mode;
    }
  }
  if (Last_Gimbal_Mode == GIMBAL_ZERO_FORCE && Gimbal_Mode != GIMBAL_ZERO_FORCE) {
    Gimbal_Mode = GIMBAL_INITIAL;
    gimbal_InitialFlag = 1;
  }
}
void gimbal_calculate()
{
  yaw_angle.relative = sp::limit_angle(yaw_motor.angle - yaw_angle.installed);
  pitch_angle.relative = sp::limit_angle(pitch_motor.angle - pitch_angle.installed);
  switch (Gimbal_Mode) {
    case GIMBAL_ZERO_FORCE:
      break;
    case GIMBAL_INITIAL:
      gimbal_initial();
      break;
    case GIMBAL_GYRO:
      gimbal_gyro();
      break;
    case GIMBAL_AUTO:
      gimbal_auto();
      break;
    case GIMBAL_LOB:
      gimbal_lob();
      break;
  }
}

void gimbal_initial()
{
  static uint16_t gimbal_init_time = 0;
  static uint16_t gimbal_init_over_time = 0;

  if (gimbal_InitialFlag) {
    //云台回正，与底盘没有相对角度
    yaw_angle.target = 0.0f;
    pitch_angle.target = 0.0f;

    // 判断是否已回中
    if (fabs(yaw_angle.relative) < 0.05f && fabs(pitch_angle.relative) < 0.05f) {
      gimbal_init_over_time++;
    }

    gimbal_init_time++;

    // 第二步：满足条件则初始化完成
    if (gimbal_init_over_time >= 500 || gimbal_init_time >= 1000) {
      // 把当前 IMU 姿态写为新的目标角，变换进imu坐标系
      yaw_angle.target = imu.yaw;
      pitch_angle.target = imu.pitch;

      gimbal_init_over_time = 0;
      gimbal_init_time = 0;
      gimbal_InitialFlag = 0;

      Last_Gimbal_Mode = GIMBAL_GYRO;
      Gimbal_Mode = GIMBAL_GYRO;
    }
  }
}

void gimbal_gyro()
{
  if (Global_Mode == REMOTE) {
    yaw_angle.add = -remote_yaw * W_MAX;
    pitch_angle.add = -remote_pitch * W_MAX;
    yaw_angle.target = sp::limit_angle(yaw_angle.target + yaw_angle.add);
    pitch_angle.target = sp::limit_min_max(
      sp::limit_angle(pitch_angle.target + pitch_angle.add), IMU_PITCH_ANGLE_MIN,
      IMU_PITCH_ANGLE_MAX);
  }
  if (Global_Mode == KEYBOARD) {
    yaw_angle.add = -mouse_yaw * MOUSE_DPI;
    pitch_angle.add = -mouse_pitch * MOUSE_DPI;
    yaw_angle.target = sp::limit_angle(yaw_angle.target + yaw_angle.add);
    pitch_angle.target = sp::limit_min_max(
      sp::limit_angle(pitch_angle.target + pitch_angle.add), IMU_PITCH_ANGLE_MIN,
      IMU_PITCH_ANGLE_MAX);
    if (turnover_cold_time > 0) {
      turnover_cold_time--;
    }
    if (key_yaw_left_90 && turnover_cold_time == 0) {
      yaw_angle.target += sp::PI / 2;
      turnover_cold_time = TURNOVER_COLDTIME;
    }
    if (key_yaw_right_90 && turnover_cold_time == 0) {
      yaw_angle.target += -sp::PI / 2;
      turnover_cold_time = TURNOVER_COLDTIME;
    }
    //按下X回头
    if (key_yaw_180 && turnover_cold_time == 0) {
      yaw_angle.target += sp::PI;
      turnover_cold_time = TURNOVER_COLDTIME;
    }
  }
}
void gimbal_auto()
{
  shoot_mode_flag = 1;  // 普通自瞄
  //赋予自瞄坐标
  if (vis.control) {
    yaw_angle.target = vis.yaw;
#ifdef RMUL
    pitch_angle.target = vis.pitch;
    pitch_angle.target =
      sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
    pitch_angle.target = sp::limit_min_max(
      vis.pitch, IMU_PITCH_ANGLE_MIN /*+ slope_angle*/, IMU_PITCH_ANGLE_MAX /*+ slope_angle*/);
#endif
  }
}
void gimbal_lob()
{
  yaw_angle.add = (key_lob_yaw_left ? W_MAX : 0.0f) + (key_lob_yaw_right ? W_MAX : 0.0f);
  pitch_angle.add = (key_lob_pitch_down ? W_MAX : 0.0f) + (key_lob_pitch_up ? W_MAX : 0.0f);
  yaw_angle.target = sp::limit_angle(yaw_angle.target + yaw_angle.add);
  pitch_angle.target = sp::limit_angle(pitch_angle.target + pitch_angle.add);
}
