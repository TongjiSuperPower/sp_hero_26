#ifndef MODE_HPP
#define MODE_HPP

#include "A_HERO_SELECTION.hpp"
//#define VT03

//全局基本状态机
typedef enum
{
  ZERO_FORCE,  //无力状态
  KEYBOARD,    //(PLAYER&VISION):操作手键鼠状态，视觉调试模式
  REMOTE,      //遥控器控制状态
} global_mode;

//底盘状态机
typedef enum
{
  CHASSIS_DOWN,
  CHASSIS_INIT,
  CHASSIS_MOVE,
  CHASSIS_SPIN,
  CHASSIS_FOLLOW,
} chassis_mode;

//云台状态机参数
typedef enum
{
  GIMBAL_ZERO_FORCE,  //无力状态
  GIMBAL_INIT,        //初始化回中
  GIMBAL_GYRO,        //非自瞄状态下陀螺仪控制
  GIMBAL_AUTO,        //自瞄状态下陀螺仪控制
  GIMBAL_LOB,         //吊射模式
} gimbal_mode;

//摩擦轮状态机：只有在左拨杆在中挡时允许切换，向上拨动一次即可切换摩擦轮状态
typedef enum
{
  FRIC_DOWN,  //摩擦轮关闭 发0
  FRIC_ON,    //摩擦轮打开
  FRIC_OFF,   //摩擦轮关闭 pid控
} fric_mode;

//打弹检测状态机
typedef enum
{
  FIRE_DOWN,          //摩擦轮失力状态
  FIRE_READY_DETECT,  //摩擦轮接近目标转速，可以开始检测
  FIRE_SUSPICION,     //摩擦轮掉速，怀疑射击
  FIRE_CONFIRMATION,  //摩擦轮降速一定时间，确认射击
} shoot_mode;

//拨弹轮状态机：摩擦轮开的时候才转，否则都不转
typedef enum
{
  SHOOT_DOWN,          //拨弹轮失力状态
  SHOOT_READY_SINGLE,  //拨弹轮单发模式，双环
  SHOOT_CLEAR,         //拨弹轮连发，赛后清弹(不需热量控制)
  SHOOT_INIT,          //拨弹轮复位
} trigger_mode;

// -------------------- 对外接口 --------------------
void global_mode_control();

//全局状态机
extern global_mode Global_Mode;
extern global_mode Last_Global_Mode;

//底盘状态机
extern chassis_mode Chassis_Mode;
//云台状态机
extern gimbal_mode Last_Gimbal_Mode;
extern gimbal_mode Gimbal_Mode;
//摩擦轮状态机
extern fric_mode Fric_Mode;
extern fric_mode Last_Fric_Mode;
//射击状态机
extern shoot_mode Shoot_Mode;
extern shoot_mode Last_Shoot_Mode;
//拨弹轮状态机
extern trigger_mode Trigger_Mode;
//记录反转之前的状态机
extern trigger_mode Last_Trigger_Mode;

#endif