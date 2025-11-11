#include "cmsis_os.h"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart1);

extern "C" void Plotter_Task()
{
  /* USER CODE BEGIN Plotter_Task */
  /* Infinite loop */
  while (1) {
    plotter.plot(
      trigger_motor.angle
    );
    osDelay(1);
  }
  /* USER CODE END Plotter_Task */
}