#ifndef CAN_HPP
#define CAN_HPP
#include "io/can/can.hpp"

inline sp::CAN can1(&hcan1);
inline sp::CAN can2(&hcan2);

void chassis_send();
void yaw_send();
void pitch_send();
void fric_send();
void trigger_send();

extern sp::CAN can1;
extern sp::CAN can2;
#endif