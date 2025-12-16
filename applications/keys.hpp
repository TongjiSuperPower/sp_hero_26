#include "data/uart/uart_task.hpp"
#include "io/dbus/dbus.hpp"
#include "mode.hpp"
// #include "referee/vt03/vt03.hpp"

// extern sp::VT03 vt03;

//遥控端
//底盘
extern float remote_move_y;
extern float remote_move_x;
//云台
extern float remote_yaw;
extern float remote_pitch;
//射击
extern bool remote_shoot;

//键鼠端
//底盘
extern bool key_move_y_up;
extern bool key_move_y_down;
extern bool key_move_x_up;
extern bool key_move_x_down;
extern bool key_lob_yaw_left;
extern bool key_lob_yaw_right;
extern bool key_lob_pitch_up;
extern bool key_lob_pitch_down;
extern bool key_spin;
extern bool key_cap;
//云台
extern float mouse_yaw;
extern float mouse_pitch;
extern bool key_yaw_left_90;
extern bool key_yaw_right_90;
extern bool key_yaw_180;
extern bool key_lob_mode;
//发射机构
extern bool key_shoot;
//自瞄
extern bool key_autoaim;
//刷新ui
extern bool key_ui;
//大小符
extern bool key_small_buff;
extern bool key_big_buff;
//摩擦轮转速调整
extern bool key_fric_down;
extern bool key_fric_up;