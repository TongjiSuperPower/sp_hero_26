#ifndef _CALIBRATE_TASK_H_
#define _CALIBRATE_TASK_H_
#include <cstdint>

#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "data/uart/uart_task.hpp"
#include "imu_task.hpp"
#include "mode.hpp"

// -------------------- 对外调试 --------------------
extern uint8_t calibrate_flag;
extern float gyro_x_zero;
extern float gyro_y_zero;
extern float gyro_z_zero;

extern float sum_x;
extern float sum_y;
extern float sum_z;
#endif