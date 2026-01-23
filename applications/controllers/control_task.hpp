#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "controllers/chassis_controller/chassis_task.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "io/can/can.hpp"
#include "io/vision/vision.hpp"

// inline sp::LowPassFilter chassis_follow_vx_filter(0.1f);

// -------------------- 控制参数 --------------------
//计算力矩控制参数
#ifdef HERO_DOG
// m = 0.0617, b = 0.3535, c = 0.3400
// m = 0.0716, b = 0.2665, c = 0.3082
// m = 0.0787, b = 0.2249, c = 0.3071
// m = 0.0786, b = 0.2343, c = 0.3100
//m = 0.0977, b = 0.2987, c = 0.3370
//m = 0.0958, b = 0.3693, c = 0.3699
// m = 0.0970, b = 0.2166, c = 0.3488
//m = 0.0412, b = 1.1673, c = 0.4899
// m = 0.0957, b = 0.3068, c = 0.7476
// m = 0.0922, b = 0.2150, c = 0.9411
constexpr float YAW_INERTIA = 0.0922f;
constexpr float YAW_DAMPING_COEFF = 0.2150f;
constexpr float YAW_COULOMB_FORCE = 0.1411f;

// m = 0.0331, b = 0.2112, c = 0.1037, d = 1.8725
// m = 0.0349, b = 0.2383, c = 0.0796, d = 1.8778
// m = 0.0395  b = 0.2486  c = 0.0800  d = 2.0584
//m = 0.0470, b = 0.2353, c = 0.0264, d = 1.9543
//m = 0.0468, b = 0.2527, c = 0.0107, d = 1.9504
//m = 0.0285, b = 0.3924, c = -0.0116, d = 1.8834
// m = 0.0319, b = 0.5028, c = -0.0764, d = 1.8483
constexpr float PITCH_INERTIA = 0.0395;
constexpr float PITCH_DAMPING_COEFF = 0.2486;
constexpr float PITCH_COULOMB_FORCE = 0.0800;
constexpr float PITCH_GRAVITY_TORQUE = 2.0584f;
#endif

extern float pitch_torque;
extern float a;
extern float b;
extern float c;
extern float d;
extern float e;
extern float f;
extern float g;

#endif