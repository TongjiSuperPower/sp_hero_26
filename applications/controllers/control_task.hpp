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
// m = 0.1135  b=  02295   c = 0.8333
// 0.0960 0.6715 0.3716
//0.1075 0.5423 0.4596
// m = -0.0254, b = 2.3591, c = 0.0446
// m = 0.0597, b = 0.7359, c = 0.3866
// m = 0.0079, b = 1.8003, c = 0.1164
// m = 0.0849, b = 0.5291, c = 0.4729
// constexpr float YAW_INERTIA = 0.0849f;
// constexpr float YAW_DAMPING_COEFF = 0.5291f;
// constexpr float YAW_COULOMB_FORCE = 0.4729f;
constexpr float YAW_INERTIA = 0.1336f;
constexpr float YAW_DAMPING_COEFF = 0.2605f;
constexpr float YAW_COULOMB_FORCE = 0.7021f;
// constexpr float YAW_INERTIA = 0.0597;
// constexpr float YAW_DAMPING_COEFF = 0.7359f;
// constexpr float YAW_COULOMB_FORCE = 0.3866f;

// m = 0.0331, b = 0.2112, c = 0.1037, d = 1.8725
// m = 0.0349, b = 0.2383, c = 0.0796, d = 1.8778
// m = 0.0395  b = 0.2486  c = 0.0800  d = 2.0584
//m = 0.0470, b = 0.2353, c = 0.0264, d = 1.9543
//m = 0.0468, b = 0.2527, c = 0.0107, d = 1.9504
//m = 0.0285, b = 0.3924, c = -0.0116, d = 1.8834
// m = 0.0319, b = 0.5028, c = -0.0764, d = 1.8483
// m= 0.0911  b 1.5775  -0.3289 0.1592
// m = 0.0863, b = 0.6121, c = -0.0227, d = 1.0606
// m = 0.0837, b = 0.4811, c = 0.0108, d = 0.9252
constexpr float PITCH_INERTIA = 0.0837f;
constexpr float PITCH_DAMPING_COEFF = 0.4811f;
constexpr float PITCH_COULOMB_FORCE = 0.0108f;
constexpr float PITCH_GRAVITY_TORQUE = 0.9252f; 
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