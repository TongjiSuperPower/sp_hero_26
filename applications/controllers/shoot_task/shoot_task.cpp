#include "shoot_task.hpp"

#include "cmath"
#include "cmsis_os.h"
#include "io/imu_task.hpp"
// 初始化参数
float shoot_init_time = 0.0f;
float shoot_init_over_time = 0.0f;

// 射击状态参数
bool shoot_overFlag = true;
float shoot_over_time = 0.0f;
float shoot_time = 0.0f;
float shoot_cold_time = 0.0f;

// 热量控制
float heat_remain = 0.0f;
float cal_heat = 0.0f;             // 本地计算的实时热量
uint64_t cal_cooling_count = 0;    // 冷却计时器
uint32_t shoot_count = 0;          // 射击计数
bool count_flag = true;            // 掉速检测标志位
float cur_fric_speed_avg = 0.0f;   // 当前摩擦轮平均速度
float last_fric_speed_avg = 0.0f;  // 上一次摩擦轮平均速度

// 状态机
fric_mode Fric_Mode = FRIC_DOWN;
fric_mode Last_Fric_Mode = FRIC_DOWN;
trigger_mode Trigger_Mode = TRIGGER_DOWN;

// 摩擦轮目标转速
float fric_1stSpeed_target;
float fric_2ndSpeed_target;

// 摩擦轮速度是否稳定标识符
bool fric_speedStablized_flag = false;
uint16_t fric_targetchangeCount = 500;
bool shoot_initialFlag = false;

// 拨盘电机目标
float trigger_angle_target = 0.0f;
float trigger_speed_target;
float initial_speed = 0.0f;
float last_initial_speed;

// 射击控制
static bool last_remote_shoot = false;
bool first_shoot = false;
float trigger_work_position[6];
//打弹颗数计数
float fric_speed;
float last_fric_speed;

// 函数声明
void shoot_heat_cal();  //软热量计算函数声明

extern "C" void Shoot_Task()
{
  shoot_paramInitial();
  osDelay(200);

  while (1) {
    fric_mode_control();
    trigger_mode_control();
    fric_calculate();
    shoot_heat_cal();
    trigger_calculate();

    osDelay(1);
  }
}

void shoot_paramInitial()
{
  static int last_gimbal_mode = -1;  // 记录上一次模式

  // 只有当云台模式发生改变时，才重置速度参数
  if (Gimbal_Mode != last_gimbal_mode) {
    if (Gimbal_Mode != GIMBAL_LOB) {
      fric_1stSpeed_target = FRIC_SPEED_FIRST;
      fric_2ndSpeed_target = FRIC_SPEED_SECOND;
    }
    if (Gimbal_Mode == GIMBAL_LOB) {
      fric_1stSpeed_target = FRIC_SPEED_SECOND;
      fric_2ndSpeed_target = FRIC_SPEED_SECOND_LOB;
    }
    last_gimbal_mode = Gimbal_Mode;
  }

  // 计算拨盘格点 (只需要一次)
  static bool grid_init = false;
  if (!grid_init) {
    for (int i = 0; i < 6; i++) {
      trigger_work_position[i] = sp::limit_angle(TRIGGER_INIT_ANGLE + i * sp::PI / 3);
    }
    grid_init = true;
  }
}

void fric_mode_control()
{
#ifdef DT7
  static bool last_fric_flag = (remote.sw_l == sp::DBusSwitchMode::UP);
  if (Global_Mode == ZERO_FORCE || pm02.robot_status.power_management_shooter_output == 0) {
    Fric_Mode = FRIC_DOWN;
    fric_speedStablized_flag = false;
  }
  else {
    if (remote.sw_l == sp::DBusSwitchMode::UP && !last_fric_flag /*上升沿检测*/) {
      if (Fric_Mode == FRIC_DOWN || Fric_Mode == FRIC_OFF) {
        Fric_Mode = FRIC_ON;
        first_shoot = true;
        trigger_angle_target = trigger_motor.angle;  // 保持当前位置
      }
      else {
        Fric_Mode = FRIC_OFF;
        fric_speedStablized_flag = false;
      }
    }
  }
  last_fric_flag = (remote.sw_l == sp::DBusSwitchMode::UP);
#endif
}

void fric_calculate()
{
  float first_speed =
    (fabs(fric_motor1.speed) + fabs(fric_motor2.speed) + fabs(fric_motor3.speed)) / 3.0f;
  float second_speed =
    (fabs(fric_motor4.speed) + fabs(fric_motor5.speed) + fabs(fric_motor6.speed)) / 3.0f;

  // 转速稳定后再打弹
  if (!fric_speedStablized_flag && Fric_Mode == FRIC_ON) {
    if (
      fabs(first_speed - fric_1stSpeed_target) < 5.0f &&
      fabs(second_speed - fric_2ndSpeed_target) < 5.0f) {
      fric_speedStablized_flag = true;
    }
  }

  if (Fric_Mode == FRIC_ON) {
    // 弹速检测
    last_initial_speed = initial_speed;
    initial_speed = pm02.shoot.initial_speed;

    // 最高射速限制：超射速摩擦轮自动降速
    if ((initial_speed != last_initial_speed) && pm02.shoot.initial_speed > 12.0f && key_shoot) {
      fric_1stSpeed_target -= 20.0f;
      fric_2ndSpeed_target -= 20.0f;
    }

    // 操作手手动调整摩擦轮转速
    if (pm02.game_status.game_progress == 1) {
      if (fric_targetchangeCount > 0) {
        fric_targetchangeCount--;
      }
      if (key_fric_up && fric_targetchangeCount == 0) {
        fric_targetchangeCount = 300;
        fric_1stSpeed_target += 10;
        fric_2ndSpeed_target += 10;
      }
      if (key_fric_down && fric_targetchangeCount == 0) {
        fric_targetchangeCount = 300;
        fric_1stSpeed_target -= 10;
        fric_2ndSpeed_target -= 10;
      }
    }
    Last_Fric_Mode = FRIC_ON;
  }

  if (Fric_Mode == FRIC_OFF) {
    Last_Fric_Mode = FRIC_OFF;
    fric_1stSpeed_target = 0.0f;
    fric_2ndSpeed_target = 0.0f;
    fric_speedStablized_flag = false;
  }
  if (Fric_Mode == FRIC_DOWN) {
    Last_Fric_Mode = FRIC_DOWN;
    fric_speedStablized_flag = false;
  }
}

void trigger_mode_control()
{
#ifdef DT7
  if (Global_Mode == ZERO_FORCE) {
    Trigger_Mode = TRIGGER_DOWN;
    shoot_initialFlag = false;
  }
  else if (shoot_initialFlag) {
    return;
  }
  else if (Global_Mode == REMOTE) {
    if (Fric_Mode == FRIC_ON) {
      Trigger_Mode = SHOOT_READY;
    }
    if (Fric_Mode == FRIC_OFF || Fric_Mode == FRIC_DOWN) {
      Trigger_Mode = TRIGGER_DOWN;
    }
  }
  else if (Global_Mode == KEYBOARD) {
    if (Fric_Mode == FRIC_ON) {
      Trigger_Mode = SHOOT_READY;
    }
    if (Fric_Mode == FRIC_OFF || Fric_Mode == FRIC_DOWN) {
      Trigger_Mode = TRIGGER_DOWN;
    }
  }
#endif
}

void trigger_calculate()
{
  if (Global_Mode == ZERO_FORCE) {
    trigger_motor.cmd(0.0f);
  }
  else {
    if (Trigger_Mode == TRIGGER_INITIAL) {
      trigger_initialcal();
    }
    if (Fric_Mode == FRIC_ON && Trigger_Mode == SHOOT_READY) {
      shoot_permission();
    }
  }
}

void trigger_initialcal()
{
  if (fabs(trigger_angle_target - trigger_motor.angle) < 0.05f) {
    shoot_init_over_time++;
  }
  shoot_init_time++;

  if (shoot_init_time == 1000 || shoot_init_over_time > 50) {
    shoot_init_time = 0;
    shoot_init_over_time = 0;
    shoot_initialFlag = false;
  }
}

//热量计算
void shoot_heat_cal()
{
  // 获取裁判系统冷却参数
  float cooling_per_sec = (float)(pm02.robot_status.shooter_barrel_cooling_value);
  float heat_per_shot = HEAT_PRE_SHOT;

  //计算摩擦轮平均速度 (用于检测掉速)
  last_fric_speed_avg = cur_fric_speed_avg;
  cur_fric_speed_avg = (fabs(fric_motor1.speed) + fabs(fric_motor2.speed)) / 2.0f;

  // 掉速检测
  if (Fric_Mode == FRIC_ON && pm02.robot_status.power_management_shooter_output) {
    if (last_fric_speed_avg - cur_fric_speed_avg > 20.0f && count_flag == false) {
      shoot_count++;
      cal_heat += heat_per_shot;
      count_flag = true;
    }
    else {
      count_flag = false;
    }
  }

  // 冷却逻辑
  if (cal_heat > 0) {
    cal_cooling_count++;
    if (cal_cooling_count % 100 == 99) {
      cal_heat -= cooling_per_sec / 10.0f;  // 减去 0.1秒 的冷却量
    }
  }
  if (cal_heat <= 0) {
    cal_heat = 0;
    shoot_count = 0;
    cal_cooling_count = 0;
  }
}

void shoot_permission()
{
  if (!shoot_overFlag) {
    // 计算上一次射击完成时间
    if (fabs(trigger_angle_target - trigger_motor.angle) < 0.04f) {
      shoot_over_time++;
    }
    // 计算上一次射击持续时间
    shoot_time++;
  }

  // 判断上一次射击是否完成
  if (shoot_over_time > 50 || shoot_time > 149) {
    shoot_overFlag = true;
    shoot_over_time = 0;
    shoot_time = 0;
  }

  // 上次射击完成则开始减少冷却
  if (shoot_overFlag) {
    if (shoot_cold_time > 0) {
      shoot_cold_time--;
    }
  }
  float heat_limit = pm02.robot_status.shooter_barrel_heat_limit;
  bool heat_ok = (cal_heat < (heat_limit - HEAT_PRE_SHOT));

  // SHOOT_READY 射击条件
  if (Global_Mode == REMOTE) {
    bool cur_remote_shoot = remote_shoot;
    bool remote_edge = (cur_remote_shoot && !last_remote_shoot);
    if (Fric_Mode == FRIC_ON && remote_edge && shoot_cold_time == 0 && heat_ok) {
      if (first_shoot) {
        trigger_angle_target = trigger_near_work_position();
        first_shoot = false;
      }
      else {
        // 原V1逻辑：trigger_angle_target = trigger_motor.angle - SHOOT_ANGLE_ADD;
        // 建议修改为基于格点计算，如下：
        trigger_angle_target = sp::limit_angle(trigger_angle_target - sp::PI / 3);
      }

      shoot_cold_time = SHOOT_COLD_TIME;
      shoot_overFlag = false;
    }
    last_remote_shoot = cur_remote_shoot;
  }

  if (Global_Mode == KEYBOARD) {
    if (
      Fric_Mode == FRIC_ON &&
      (key_shoot || (vis.fire && vis.control && Gimbal_Mode == GIMBAL_AUTO)) &&
      shoot_cold_time == 0 && heat_ok) {  // 使用 heat_ok 替代 heat_remain

      if (first_shoot == true) {
        trigger_angle_target = trigger_near_work_position();
        first_shoot = false;
      }
      else {
        trigger_angle_target = sp::limit_angle(trigger_angle_target - sp::PI / 3);
      }
      shoot_cold_time = SHOOT_COLD_TIME;
      shoot_overFlag = false;
    }
  }
}

float trigger_near_work_position(void)
{
  float min_distance = sp::PI / 3;
  float target_position = trigger_motor.angle;
  for (int i = 0; i < 6; i++) {
    float distance = trigger_motor.angle - trigger_work_position[i];
    if (distance >= 0.0f && distance <= min_distance) {
      target_position = trigger_work_position[i];
      break;
    }
  }
  return target_position;
}