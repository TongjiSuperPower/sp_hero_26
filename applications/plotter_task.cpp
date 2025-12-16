#include "cmsis_os.h"
#include "control_task.hpp"
#include "controllers/gimbal_task/gimbal_task.hpp"
#include "controllers/shoot_task/shoot_task.hpp"
#include "io/imu_task.hpp"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart1);

extern "C" void Plotter_Task()
{
  /* USER CODE BEGIN Plotter_Task */
  /* Infinite loop */
  while (1) {
    plotter.plot(yaw_angle.target,imu.yaw);
    osDelay(10);
  }
  /* USER CODE END Plotter_Task */
}