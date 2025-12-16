#include "keys.hpp"

#include "cmsis_os.h"

//遥控端
//底盘
float remote_move_y;
float remote_move_x;
//云台
float remote_yaw;
float remote_pitch;
//射击
bool remote_shoot;

//键鼠端
//底盘
bool key_move_y_up;
bool key_move_y_down;
bool key_move_x_up;
bool key_move_x_down;
bool key_spin = false;
bool key_cap;
bool key_lob_yaw_left;
bool key_lob_yaw_right;
bool key_lob_pitch_up;
bool key_lob_pitch_down;
//云台
float mouse_yaw;
float mouse_pitch;
bool key_yaw_left_90;
bool key_yaw_right_90;
bool key_yaw_180;
bool key_lob_mode;
//发射机构
bool key_shoot;
//自瞄
bool key_autoaim;
//刷新ui
bool key_ui;
//摩擦轮转速调整
bool key_fric_down;
bool key_fric_up;

extern "C" void Keys_Task()
{
  while (1) {
    //     /#ifdef VT03
    //     //遥控端
    //     //底盘
    //     remote_move_y = vt03.ch_rh;
    //     remote_move_x = vt03.ch_rv;
    //     //云台
    //     remote_yaw = vt03.ch_lh;
    //     remote_pitch = vt03.ch_lv;
    //     //射击
    //     remote_shoot = vt03.trigger;

    //     //键鼠端
    //     //底盘
    //     key_move_x_up = vt03.keys.w;
    //     key_move_x_down = vt03.keys.s;
    //     key_move_y_up = vt03.keys.a;
    //     key_move_y_down = vt03.keys.d;
    //     key_spin = vt03.keys.shift;
    //     key_cap = vt03.keys.c;
    //     //云台
    //     mouse_yaw = vt03.mouse.vx;
    //     mouse_pitch = vt03.mouse.vy;
    //     key_yaw_left_90 = vt03.keys.q;
    //     key_yaw_right_90 = vt03.keys.e;
    //     key_yaw_180 = vt03.keys.x;
    //     key_lob_shot = vt03.keys.ctrl;
    //     //发射机构
    //     key_shoot = vt03.mouse.left;
    //     //自瞄
    //     key_autoaim = vt03.mouse.right;
    //     //刷新ui
    //     key_ui = vt03.keys.r;
    //     //大小符
    //     key_small_buff = vt03.keys.v;
    //     key_big_buff = vt03.keys.b;
    //     //摩擦轮转速调整
    //     key_fric_down = vt03.keys.v;
    //     key_fric_up = vt03.keys.b;
    //     osDelay(1);
    // #endif

#ifdef DT7
    //遥控端
    //底盘
    remote_move_y = remote.ch_lh;
    remote_move_x = remote.ch_lv;
    //云台
    remote_yaw = remote.ch_rh;
    remote_pitch = remote.ch_rv;
    //射击
    remote_shoot = (remote.sw_l == sp::DBusSwitchMode::DOWN);

    //键鼠端
    //底盘
    key_move_x_up = remote.keys.w;
    key_move_x_down = remote.keys.s;
    key_move_y_up = remote.keys.a;
    key_move_y_down = remote.keys.d;
    key_lob_pitch_up = remote.keys.w;
    key_lob_pitch_down = remote.keys.s;
    key_lob_yaw_left = remote.keys.a;
    key_lob_yaw_right = remote.keys.d;
    key_spin = remote.keys.shift;
    key_cap = remote.keys.c;
    //云台
    mouse_yaw = remote.mouse.vx;
    mouse_pitch = remote.mouse.vy;
    key_yaw_left_90 = remote.keys.q;
    key_yaw_right_90 = remote.keys.e;
    key_yaw_180 = remote.keys.x;
    key_lob_mode = remote.keys.ctrl;
    //发射机构
    key_shoot = remote.mouse.left;
    //自瞄
    key_autoaim = remote.mouse.right;
    //刷新ui
    key_ui = remote.keys.r;
    //摩擦轮转速调整
    key_fric_down = remote.keys.v;
    key_fric_up = remote.keys.b;
    osDelay(1);
#endif
  }
}