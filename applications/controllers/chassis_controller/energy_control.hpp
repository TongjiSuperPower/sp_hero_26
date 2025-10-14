#ifndef ENERGY_CONTROL_HPP
#define ENERGY_CONTROL_HPP
#include "A_HERO_SELECTION.hpp"
#include "chassis_task.hpp"

void chassis_energy_control(Wheel_Speed wheel_speed, Wheel_Torque * given_torque);
extern float kers_torque_scale;
#endif