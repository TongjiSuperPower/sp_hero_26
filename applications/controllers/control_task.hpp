#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "controllers/chassis_controller/chassis_task.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "io/can/can.hpp"
#include "io/vision/vision.hpp"

// -------------------- 控制参数 --------------------
//计算力矩控制参数
#ifdef HERO_DOG
// m = 0.0617, b = 0.3535, c = 0.3400
constexpr float YAW_INERTIA = 0.0617f;
constexpr float YAW_DAMPING_COEFF = 0.3535f;
constexpr float YAW_COULOMB_FORCE = 0.3400f;

// m = 0.0057, b = -0.6656, c = 0.0461, d = -1.0432
// m = 0.0197, b = 0.4528, c = 0.0081, d = 1.7668
// m = -0.0386, b = 0.9088, c = 0.0221, d = 1.7987
// m = -0.0365, b = 0.9549, c = 0.0068, d = 1.7802
constexpr float PITCH_INERTIA = -0.0365f;
constexpr float PITCH_DAMPING_COEFF = 0.9549f;
constexpr float PITCH_COULOMB_FORCE = 0.0068f;
constexpr float PITCH_GRAVITY_TORQUE = 1.7802f;
#endif

extern float pitch_torque;

#endif