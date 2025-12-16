#include "uart_task.hpp"
bool game_start_flag = false;
extern "C" void UART_Task()
{
  pm02.request();
  remote.request();
  while (true) {
    osDelay(10);
  }
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
  if (huart == &huart6) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  if (huart == &huart6) {
    pm02.request();
  }
}