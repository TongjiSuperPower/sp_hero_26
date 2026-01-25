#include "gimbal_task.hpp"

#include "cmsis_os.h"
#include "controllers/keys.hpp"
#include "controllers/mode.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "io/imu_task.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "controllers/chassis_controller/chassis_task.hpp"

//上坡角度
//pitch相对角度滤波
sp::LowPassFilter pitch_relative_angle_filter(0.1f);
extern float slope_angle;

//云台回中模式下回中后的时间
uint16_t gimbal_init_over_time = 0;
//云台进入回中模式的时间
uint16_t gimbal_init_time = 0;
//云台是否在回中模式
uint8_t gimbal_init_flag = 0;
//掉头冷却时间1s
uint16_t turnover_cold_time = TURNOVER_COLDTIME;
//发送给上位机的射击状态标识符
uint8_t shoot_mode_flag = 0;

//舵机位置控制
float servo_position;

bool last_key_lob_mode = false;
bool last_key_autoaim = false;

// LOB 模式的增量步进控制
constexpr float LOB_YAW_STEP = 0.005f;      // yaw 每次转动的固定角度（弧度）
constexpr float LOB_PITCH_STEP = 0.001f;   // pitch 每次转动的固定角度（弧度）

// 记录上一次按键状态（用于检测上升沿）
static bool last_key_move_y_up = false;
static bool last_key_move_y_down = false;
static bool last_key_move_x_up = false;
static bool last_key_move_x_down = false;

//云台初始化
void gimbal_init();
//云台状态选择
void gimbal_mode_control();
//云台电流解算
void gimbal_cmd();
//舵机控制
void servo_cmd();

//变量们
//当中码盘值等效换算的角度
//区间：-Π~Π
float yaw_offecd_ecd_angle = 0.0f;
float pitch_offecd_ecd_angle = 0.0f;
//yaw解算的当前码盘值相对于正中码盘值的差
float yaw_relative_angle = 0.0f;
//pit解算的当前码盘值相对于正中码盘值的差
float pitch_relative_angle = 0.0f;
//yaw，pitch目标参数
float gyro_yaw_angle_add = 0.0f;
float gyro_pitch_angle_add = 0.0f;
float yaw_target_angle = 0.0f;
float pitch_target_angle = 0.0f;
//小陀螺补偿
extern sp::Mecanum chassis;
//输入给yaw的扭矩
float yaw_cmd_torque = 0.0f;

extern "C" void Gimbal_Task()
{
  osDelay(700);  // 等待各个任务初始化完成
  //云台初始化
  gimbal_init();
  //射击初始化
  shoot_mode_init();

  while (1) {
    //云台电机选择模式
    gimbal_mode_control();
    //计算云台目标速度
    gimbal_cmd();
    //摩擦轮选择模式
    fric_mode_control();
    //拨弹轮选择模式
    trigger_mode_control();
    //计算摩擦轮目标速度
    fric_cmd();
    //计算拨弹轮目标角度
    trigger_cmd();
    //舵机控制
    servo_cmd();

    osDelay(1);
  }
}

//云台初始化（被打死之后上电回中）
void gimbal_init()
{
#ifdef HERO_DOG
  yaw_offecd_ecd_angle = 2.79260731f;
  pitch_offecd_ecd_angle = 0.62146f;
#endif
#ifdef HERO_THREE_WHEELS
  yaw_offecd_ecd_angle = 2.3814f;
  pitch_offecd_ecd_angle = -0.780f;
#endif
}

//云台状态选择
// void gimbal_mode_control()
// {
//   //正在回中过程中无法调整模式
//   if (gimbal_init_flag == 1) {
//     return;
//   }

//   //DOWN
//   if (Global_Mode == ZERO_FORCE) {
//     Last_Gimbal_Mode = Gimbal_Mode;
//     Gimbal_Mode = GIMBAL_ZERO_FORCE;
//     shoot_mode_flag = 0;
//   }

//   //遥控器模式
//   if (Global_Mode == REMOTE) {
//     Last_Gimbal_Mode = Gimbal_Mode;
//     Gimbal_Mode = GIMBAL_GYRO;
//     shoot_mode_flag = 0;
//   }

//   键鼠 长按鼠标右键自瞄
//   if (Global_Mode == KEYBOARD) {
//     Last_Gimbal_Mode = Gimbal_Mode;
//     Gimbal_Mode = GIMBAL_GYRO;
//     // 检测 Ctrl 按键上升沿（未按 → 按下）
//     if (key_lob_mode && !last_key_lob_mode) {
//       // 上升沿触发：在 LOB 和 GYRO 之间切换
//       Last_Gimbal_Mode = Gimbal_Mode;
//       if (Gimbal_Mode == GIMBAL_LOB) {
//         Gimbal_Mode = GIMBAL_GYRO;
//       }
//       else {
//         Gimbal_Mode = GIMBAL_LOB;
//       }
//       shoot_mode_flag = 0;
//       last_key_lob_mode = remote.keys.ctrl;
//     }
//     if(key_autoaim && !last_key_autoaim) {
//       Last_Gimbal_Mode = Gimbal_Mode;
//       Gimbal_Mode = GIMBAL_AUTO;
//       shoot_mode_flag = 1;
//       last_key_autoaim = remote.mouse.right;
//     }
//     else if(!key_autoaim && last_key_autoaim){
      
//       Gimbal_Mode = Last_Gimbal_Mode;
//       shoot_mode_flag = 0;
//       last_key_autoaim = remote.mouse.right;
//     }
//     // // 自瞄优先级最高
//     // else if (key_autoaim) {
//     //   Last_Gimbal_Mode = Gimbal_Mode;
//     //   Gimbal_Mode = GIMBAL_AUTO;
//     //   shoot_mode_flag = 1;
//     // }
//     // // 默认陀螺仪模式
//     // else {
//     //   Last_Gimbal_Mode = Gimbal_Mode;
//     //   Gimbal_Mode = GIMBAL_GYRO;
//     //   shoot_mode_flag = 0;
//     // }
// #ifdef VISION
//     Gimbal_Mode = GIMBAL_AUTO;
// #endif
//   }
//   //判断是否进入回中模式
//   if (Last_Gimbal_Mode == GIMBAL_ZERO_FORCE && Gimbal_Mode != GIMBAL_ZERO_FORCE) {
//     Gimbal_Mode = GIMBAL_INIT;
//     gimbal_init_flag = 1;
//     shoot_mode_flag = 0;
//   }


// #ifdef DUAL_PID
//   yaw_autoaim_pos_pid.data.iout = 0.0f;
// #endif
// }
void gimbal_mode_control()
{
    // --- 1. 定义静态变量保持状态 ---
    // lob_enable_flag: 记录当前是否处于 LOB (吊射) 开启状态
    static bool lob_enable_flag = 0; 
    // last_key_lob: 用于检测按键按下瞬间
    static bool last_key_lob = 0;

    // 正在回中过程中无法调整模式
    if (gimbal_init_flag == 1) {
        return;
    }

    // DOWN (掉电/无力模式)
    if (Global_Mode == ZERO_FORCE) {
        Last_Gimbal_Mode = Gimbal_Mode;
        Gimbal_Mode = GIMBAL_ZERO_FORCE;
        shoot_mode_flag = 0;
    }

    // 遥控器模式
    if (Global_Mode == REMOTE) {
        Last_Gimbal_Mode = Gimbal_Mode;
        Gimbal_Mode = GIMBAL_GYRO;
        shoot_mode_flag = 0;
    }

    // --- KEYBOARD 模式核心逻辑 ---
    if (Global_Mode == KEYBOARD) {

        // ============================================
        // 步骤 1：检测 LOB 开关按键 (Toggle)
        // ============================================
        // 检测上升沿：按下的瞬间翻转标志位
        if (key_lob_mode && !last_key_lob) {
            lob_enable_flag = !lob_enable_flag;
        }
        last_key_lob = key_lob_mode; // 更新按键历史


        // ============================================
        // 步骤 2：根据 [开关状态] + [自瞄按键] 决定最终模式
        // ============================================
        Last_Gimbal_Mode = Gimbal_Mode; // 记录上一时刻模式

        if (key_autoaim) {
            // --- 按住鼠标右键 (自瞄激活) ---

            if (lob_enable_flag) {
                // 如果 LOB 开关是开的 -> 进入【吊射自瞄】
                Gimbal_Mode = GIMBAL_LOB_AUTO;
            } else {
                // 如果 LOB 开关是关的 -> 进入【普通自瞄】
                Gimbal_Mode = GIMBAL_AUTO;
            }
        } 
        else {
            // --- 松开鼠标右键 (手动控制) ---

            if (lob_enable_flag) {
                // 如果 LOB 开关是开的 -> 回到【吊射模式】
                Gimbal_Mode = GIMBAL_LOB;
                // 注意：这里没有置零 shoot_mode_flag，默认吊射模式允许发射
            } else {
                // 如果 LOB 开关是关的 -> 回到【普通陀螺仪】
                Gimbal_Mode = GIMBAL_GYRO;
                // 普通巡逻模式下，通常为了安全会重置发射允许标志
                shoot_mode_flag = 0;
            }
        }
    }

    // 判断是否进入回中模式
    if (Last_Gimbal_Mode == GIMBAL_ZERO_FORCE && Gimbal_Mode != GIMBAL_ZERO_FORCE) {
        Gimbal_Mode = GIMBAL_INIT;
        gimbal_init_flag = 1;
        shoot_mode_flag = 0;
    }

#ifdef DUAL_PID
    yaw_autoaim_pos_pid.data.iout = 0.0f;
#endif
}

void gimbal_cmd()
{
  yaw_relative_angle = sp::limit_angle(yaw_motor.angle - yaw_offecd_ecd_angle);
  pitch_relative_angle = sp::limit_angle(pitch_motor.angle - pitch_offecd_ecd_angle);
  pitch_relative_angle_filter.update(pitch_relative_angle);
  pitch_relative_angle = pitch_relative_angle_filter.out;

  //云台模式为GYRO
  if (Gimbal_Mode == GIMBAL_GYRO) {
    //遥控器
    if (Global_Mode == REMOTE) {
      gyro_yaw_angle_add = -remote_yaw * W_MAX;
      gyro_pitch_angle_add = -remote_pitch * W_MAX;
      yaw_target_angle = sp::limit_angle(yaw_target_angle + gyro_yaw_angle_add);
      pitch_target_angle = sp::limit_angle(pitch_target_angle + gyro_pitch_angle_add);
      //pitch轴限角
#ifdef RMUL
      pitch_target_angle =
        sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
      pitch_target_angle = sp::limit_min_max(
        pitch_target_angle, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
    }
    //键鼠
    if (Global_Mode == KEYBOARD) {
      //陀螺仪控云台
      gyro_yaw_angle_add = -mouse_yaw * MOUSE_DPI;
      gyro_pitch_angle_add = -mouse_pitch * MOUSE_DPI;

      yaw_target_angle = sp::limit_angle(yaw_target_angle + gyro_yaw_angle_add);
      pitch_target_angle = sp::limit_angle(pitch_target_angle + gyro_pitch_angle_add);
      //掉头冷却减少
      if (turnover_cold_time > 0) {
        turnover_cold_time--;
      }
      if (key_yaw_left_90 && turnover_cold_time == 0 && !remote.keys.c) {
        yaw_target_angle += sp::PI / 2;
        turnover_cold_time = TURNOVER_COLDTIME;
      }
      if (key_yaw_right_90 && turnover_cold_time == 0 && !remote.keys.c) {
        yaw_target_angle += -sp::PI / 2;
        turnover_cold_time = TURNOVER_COLDTIME;
      }
      //按下X回头
      if (key_yaw_180 && turnover_cold_time == 0) {
        yaw_target_angle += sp::PI;
        turnover_cold_time = TURNOVER_COLDTIME;
      }
      //pitch轴限角
#ifdef RMUL
      pitch_target_angle =
        sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
      pitch_target_angle = sp::limit_min_max(
        pitch_target_angle, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
    }
  }

  //自瞄控云台
  if (Gimbal_Mode == GIMBAL_AUTO) {
    shoot_mode_flag = 1;  // 普通自瞄
    //赋予自瞄坐标
    if (vis.control) {
      yaw_target_angle = vis.yaw;
#ifdef RMUL
      pitch_target_angle = vis.pitch;
      pitch_target_angle =
        sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
      pitch_target_angle = sp::limit_min_max(
        vis.pitch, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
    }

  }

  if(Gimbal_Mode == GIMBAL_LOB){
    // W键 - Pitch 抬起（pitch 减小）
    if(key_move_x_up && !last_key_move_x_up){
        pitch_target_angle -= LOB_PITCH_STEP;
    }
    // S键 - Pitch 放下（pitch 增大）
    if(key_move_x_down && !last_key_move_x_down){
        pitch_target_angle += LOB_PITCH_STEP;
    }
    // A键 - Yaw 左转
    if(key_move_y_up && !last_key_move_y_up){
        yaw_target_angle += LOB_YAW_STEP;
    }
    // D键 - Yaw 右转
    if(key_move_y_down && !last_key_move_y_down){
        yaw_target_angle -= LOB_YAW_STEP;
    }
    
    // 更新上一次按键状态
    last_key_move_x_up = key_move_x_up;
    last_key_move_x_down = key_move_x_down;
    last_key_move_y_up = key_move_y_up;
    last_key_move_y_down = key_move_y_down;
    
    // 应用角度限制
    #ifdef RMUL
    pitch_target_angle = sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
    #endif
    #ifdef RMUC
    pitch_target_angle = sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
    #endif
}
  if(Gimbal_Mode == GIMBAL_LOB_AUTO){
  
    //赋予自瞄坐标
      // yaw_target_angle = vis.yaw;
      pitch_target_angle = vis.pitch;
    }

  if (Gimbal_Mode == GIMBAL_INIT) {
    yaw_target_angle = 0.0f;
    pitch_target_angle = 0.0f;
    if ((fabs(yaw_relative_angle)) < 0.05f && (fabs(pitch_relative_angle)) < 0.05f) {
      gimbal_init_over_time++;
    }
    gimbal_init_time++;

    //判断初始化完成
    if (gimbal_init_time == 1000 || gimbal_init_over_time == 500) {
      yaw_target_angle = imu.yaw;
      pitch_target_angle = imu.pitch;
      gimbal_init_over_time = 0;
      gimbal_init_time = 0;
      gimbal_init_flag = false;
    }
  }
}

void servo_cmd()
{
  static uint8_t last_key_q = 0;
  static uint8_t last_key_e = 0;

  // 按住C键时，检测Q键按下边缘（从未按下到按下的瞬间）
  if (remote.keys.c && remote.keys.q && !last_key_q) {
    servo_position += 2.0f;  // 增加一个小角度
  }
  // 按住C键时，检测E键按下边缘（从未按下到按下的瞬间）
  else if (remote.keys.c && remote.keys.e && !last_key_e) {
    servo_position -= 2.0f;  // 减少一个小角度
  }

  // 更新上一次的按键状态
  last_key_q = remote.keys.q;
  last_key_e = remote.keys.e;
}
