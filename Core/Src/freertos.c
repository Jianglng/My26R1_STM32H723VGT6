/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t configured;         /* USB enumeration state, refreshed by defaultTask. */
  uint32_t has_frame;          /* At least one checksum-valid, finite pose received. */
  uint32_t frame_count;
  uint32_t checksum_error_count;
  uint32_t invalid_value_count;
  uint32_t last_valid_tick;    /* HAL_GetTick(), milliseconds. */
  uint32_t last_chunk_len;
  float x;                    /* World X, metres. */
  float y;                    /* World Y, metres. */
  float yaw;                  /* Normalized to [-pi, pi], radians. */
  float yaw_raw;
  uint8_t last_frame[15];      /* Last accepted protocol frame. */
  uint8_t last_chunk[128];     /* Last bytes read by the task, including bad data. */
} USB_PoseDebugTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_POSE_FRAME_SIZE 15U
#define USB_POSE_PI 3.14159265358979323846f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern USBD_HandleTypeDef hUsbDeviceHS;
/* Add these two globals to Keil Watch: g_usb_pose_debug, g_usb_rx_debug. */
volatile USB_PoseDebugTypeDef g_usb_pose_debug;
static uint8_t usb_task_rx[128];
static uint8_t usb_pose_frame[USB_POSE_FRAME_SIZE];
static uint8_t usb_pose_frame_len;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void USB_PoseFeedByte(uint8_t byte);
static uint8_t USB_PoseDecodeFloat(const uint8_t *bytes, float *value);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartChassisTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(StartChassisTask, NULL, &Chassis_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    uint8_t stream_reset;
    uint16_t i;
    uint16_t len = CDC_Read_HS(usb_task_rx, sizeof(usb_task_rx), &stream_reset);

    g_usb_pose_debug.configured = (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED);
    if (stream_reset != 0U)
    {
      usb_pose_frame_len = 0U;
    }
    if (len != 0U)
    {
      g_usb_pose_debug.last_chunk_len = len;
      for (i = 0U; i < len; i++)
      {
        g_usb_pose_debug.last_chunk[i] = usb_task_rx[i];
        USB_PoseFeedByte(usb_task_rx[i]);
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
__weak void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* The wire format is little-endian IEEE754 float32, with no struct padding. */
static uint8_t USB_PoseDecodeFloat(const uint8_t *bytes, float *value)
{
  uint32_t bits = (uint32_t)bytes[0]
                | ((uint32_t)bytes[1] << 8)
                | ((uint32_t)bytes[2] << 16)
                | ((uint32_t)bytes[3] << 24);
  if ((bits & 0x7F800000UL) == 0x7F800000UL)
  {
    return 0U; /* Reject NaN/Inf before yaw normalization. */
  }
  memcpy(value, &bits, sizeof(bits));
  return 1U;
}

static void USB_PoseFeedByte(uint8_t byte)
{
  uint8_t i;
  uint8_t checksum = 0U;
  float x;
  float y;
  float yaw;
  float yaw_raw;

  usb_pose_frame[usb_pose_frame_len++] = byte;
  /* Sliding search also handles AA AA 55, noise and headers inside bad frames. */
  while ((usb_pose_frame_len >= 2U)
         && ((usb_pose_frame[0] != 0xAAU) || (usb_pose_frame[1] != 0x55U)))
  {
    usb_pose_frame_len--;
    memmove(usb_pose_frame, usb_pose_frame + 1, usb_pose_frame_len);
  }
  if (usb_pose_frame_len != USB_POSE_FRAME_SIZE)
  {
    return;
  }

  for (i = 0U; i < USB_POSE_FRAME_SIZE - 1U; i++)
  {
    checksum = (uint8_t)(checksum + usb_pose_frame[i]);
  }
  if (checksum != usb_pose_frame[USB_POSE_FRAME_SIZE - 1U])
  {
    g_usb_pose_debug.checksum_error_count++;
    usb_pose_frame_len--;
    memmove(usb_pose_frame, usb_pose_frame + 1, usb_pose_frame_len);
    return;
  }
  usb_pose_frame_len = 0U;
  if ((USB_PoseDecodeFloat(usb_pose_frame + 2, &x) == 0U)
      || (USB_PoseDecodeFloat(usb_pose_frame + 6, &y) == 0U)
      || (USB_PoseDecodeFloat(usb_pose_frame + 10, &yaw_raw) == 0U))
  {
    g_usb_pose_debug.invalid_value_count++;
    return;
  }

  yaw = fmodf(yaw_raw, 2.0f * USB_POSE_PI);
  if (yaw > USB_POSE_PI)
  {
    yaw -= 2.0f * USB_POSE_PI;
  }
  else if (yaw < -USB_POSE_PI)
  {
    yaw += 2.0f * USB_POSE_PI;
  }
  g_usb_pose_debug.x = x;
  g_usb_pose_debug.y = y;
  g_usb_pose_debug.yaw = yaw;
  g_usb_pose_debug.yaw_raw = yaw_raw;
  for (i = 0U; i < USB_POSE_FRAME_SIZE; i++)
  {
    g_usb_pose_debug.last_frame[i] = usb_pose_frame[i];
  }
  g_usb_pose_debug.last_valid_tick = HAL_GetTick();
  g_usb_pose_debug.has_frame = 1U;
  g_usb_pose_debug.frame_count++;
}
/* USER CODE END Application */

