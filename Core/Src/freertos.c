/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId chassis_taskHandle;
osThreadId led_taskHandle;
osThreadId uart_taskHandle;
osThreadId gimbal_taskHandle;
osThreadId imu_taskHandle;
osThreadId buzzer_taskHandle;
osThreadId calibrate_taskHandle;
osThreadId plotter_taskHandle;
osThreadId control_taskHandle;
osThreadId detect_taskHandle;
osThreadId keys_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Chassis_Task(void const * argument);
void LED_Task(void const * argument);
void UART_Task(void const * argument);
void Gimbal_Task(void const * argument);
void IMU_task(void const * argument);
void Buzzer_Task(void const * argument);
void Calibrate_Task(void const * argument);
void Plotter_Task(void const * argument);
void Control_Task(void const * argument);
void Detect_Task(void const * argument);
void Keys_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of chassis_task */
  osThreadDef(chassis_task, Chassis_Task, osPriorityAboveNormal, 0, 512);
  chassis_taskHandle = osThreadCreate(osThread(chassis_task), NULL);

  /* definition and creation of led_task */
  osThreadDef(led_task, LED_Task, osPriorityLow, 0, 128);
  led_taskHandle = osThreadCreate(osThread(led_task), NULL);

  /* definition and creation of uart_task */
  osThreadDef(uart_task, UART_Task, osPriorityAboveNormal, 0, 128);
  uart_taskHandle = osThreadCreate(osThread(uart_task), NULL);

  /* definition and creation of gimbal_task */
  osThreadDef(gimbal_task, Gimbal_Task, osPriorityAboveNormal, 0, 512);
  gimbal_taskHandle = osThreadCreate(osThread(gimbal_task), NULL);

  /* definition and creation of imu_task */
  osThreadDef(imu_task, IMU_task, osPriorityHigh, 0, 1024);
  imu_taskHandle = osThreadCreate(osThread(imu_task), NULL);

  /* definition and creation of buzzer_task */
  osThreadDef(buzzer_task, Buzzer_Task, osPriorityLow, 0, 256);
  buzzer_taskHandle = osThreadCreate(osThread(buzzer_task), NULL);

  /* definition and creation of calibrate_task */
  osThreadDef(calibrate_task, Calibrate_Task, osPriorityBelowNormal, 0, 128);
  calibrate_taskHandle = osThreadCreate(osThread(calibrate_task), NULL);

  /* definition and creation of plotter_task */
  osThreadDef(plotter_task, Plotter_Task, osPriorityLow, 0, 128);
  plotter_taskHandle = osThreadCreate(osThread(plotter_task), NULL);

  /* definition and creation of control_task */
  osThreadDef(control_task, Control_Task, osPriorityAboveNormal, 0, 1024);
  control_taskHandle = osThreadCreate(osThread(control_task), NULL);

  /* definition and creation of detect_task */
  osThreadDef(detect_task, Detect_Task, osPriorityNormal, 0, 128);
  detect_taskHandle = osThreadCreate(osThread(detect_task), NULL);

  /* definition and creation of keys_task */
  osThreadDef(keys_task, Keys_Task, osPriorityNormal, 0, 128);
  keys_taskHandle = osThreadCreate(osThread(keys_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_LED_Task */
/**
* @brief Function implementing the led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Task */
__weak void LED_Task(void const * argument)
{
  /* USER CODE BEGIN LED_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_Task */
}

/* USER CODE BEGIN Header_UART_Task */
/**
* @brief Function implementing the uart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Task */
__weak void UART_Task(void const * argument)
{
  /* USER CODE BEGIN UART_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the gimbal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_IMU_task */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_task */
__weak void IMU_task(void const * argument)
{
  /* USER CODE BEGIN IMU_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_task */
}

/* USER CODE BEGIN Header_Buzzer_Task */
/**
* @brief Function implementing the buzzer_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Task */
__weak void Buzzer_Task(void const * argument)
{
  /* USER CODE BEGIN Buzzer_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Buzzer_Task */
}

/* USER CODE BEGIN Header_Calibrate_Task */
/**
* @brief Function implementing the calibrate_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Calibrate_Task */
__weak void Calibrate_Task(void const * argument)
{
  /* USER CODE BEGIN Calibrate_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Calibrate_Task */
}

/* USER CODE BEGIN Header_Plotter_Task */
/**
* @brief Function implementing the plotter_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Plotter_Task */
__weak void Plotter_Task(void const * argument)
{
  /* USER CODE BEGIN Plotter_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Plotter_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the control_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the detect_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* USER CODE BEGIN Header_Keys_Task */
/**
* @brief Function implementing the keys_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Keys_Task */
__weak void Keys_Task(void const * argument)
{
  /* USER CODE BEGIN Keys_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Keys_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
