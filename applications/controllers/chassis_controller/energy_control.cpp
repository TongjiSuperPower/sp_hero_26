#include "energy_control.hpp"

#include "controllers/mode.hpp"
#include "tools/math_tools/math_tools.hpp"

//电机减速扭矩pid阈值，速度大于该值，采用动能回收缓减速；速度小于此值，pid减速
constexpr float speed_limit = 0.5f;
//电机动能回收理想扭矩
Wheel_Torque kers_given_torque = {0.0f, 0.0f, 0.0f, 0.0f};
//电机理想动能回收扭矩系数
constexpr float k_kers = -11.0f * 0.007;
//电机动能回收整体衰减扭矩系数
float kers_torque_scale = 0.0f;
//是否启动动能回收：仅四个轮子均需减速时启动
bool kers_control_flag = false;

void chassis_energy_control(Wheel_Speed wheel_speed, Wheel_Torque * given_torque)
{
  if (
    (wheel_lf.speed * given_torque->lf) < 0 && (wheel_lr.speed * given_torque->lr) < 0 &&
    (wheel_rf.speed * given_torque->rf) < 0 && (wheel_rr.speed * given_torque->rr) < 0) {
    kers_control_flag = true;
  }
  else {
    kers_control_flag = false;
  }
  //电机动能回收理想扭矩
  kers_given_torque.lf = k_kers * wheel_speed.lf;
  kers_given_torque.lr = k_kers * wheel_speed.lr;
  kers_given_torque.rf = k_kers * wheel_speed.rf;
  kers_given_torque.rr = k_kers * wheel_speed.rr;

  if (kers_control_flag) {
    kers_torque_scale = std::max(
      kers_given_torque.lf / given_torque->lf,
      std::max(
        kers_given_torque.lr / given_torque->lr,
        std::max(
          kers_given_torque.rf / given_torque->rf, kers_given_torque.rr / given_torque->rr)));
    kers_torque_scale = sp::limit_min_max(kers_torque_scale, 0.0f, 1.0f);
    kers_torque_scale = 1.0f;
  }
  else {
    kers_torque_scale = 1.0f;
  }
  given_torque->lf = kers_torque_scale * given_torque->lf;
  given_torque->lr = kers_torque_scale * given_torque->lr;
  given_torque->rf = kers_torque_scale * given_torque->rf;
  given_torque->rr = kers_torque_scale * given_torque->rr;
}