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
// m = 0.0716, b = 0.2665, c = 0.3082
// m = 0.0787, b = 0.2249, c = 0.3071
// m = 0.0786, b = 0.2343, c = 0.3100
// m = 0.0924, b = 0.1947, c = 0.2668
constexpr float YAW_INERTIA = 0.0924f;
constexpr float YAW_DAMPING_COEFF = 0.1947f;
constexpr float YAW_COULOMB_FORCE = 0.2668f;

// m = 0.0331, b = 0.2112, c = 0.1037, d = 1.8725
// m = 0.0349, b = 0.2383, c = 0.0796, d = 1.8778
// m = 0.0385, b = 0.1702, c = 0.1310, d = 2.0495
// m = 0.0385, b = 0.2064, c = 0.1117, d = 2.0654
// m = 0.0384, b = 0.2107, c = 0.1121, d = 2.0786
// m = 0.0345, b = 0.3329, c = 0.0527, d = 2.0120
constexpr float PITCH_INERTIA = 0.0345f;
constexpr float PITCH_DAMPING_COEFF = 0.3329f;
constexpr float PITCH_COULOMB_FORCE = 0.0527f;
constexpr float PITCH_GRAVITY_TORQUE = 2.0120f;
#endif

extern float pitch_torque;
extern float a;
extern float b;
extern float c;
extern float d;

#endif