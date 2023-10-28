/* Custom App */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MPU9250.h"
#include "micros.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "max30102.h"
#include "algorithm_by_RF.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* Football_Player */
  uint8_t               Pag_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  uint8_t* 				Data;
  uint8_t 				TimerMeasurement_Id;

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_HEART_BEAT_TRACE 		140.0f
#define MY_SNPRINTF 		snprintf
#define FAKE_MAX30102_CLONE
#define PAGAPP_MEASUREMENT_INTERVAL   3*(1000000/CFG_TS_TICK_VAL)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

static uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
static uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data

extern MPU9250_t mpu;

extern uint8_t Rxdata[750];
char Txdata[750];
char GPS_Payyload[100];

static int Msgindex;
char *ptr;

int Time_before, Time_after;
int Latitude_before, Latitude_after;
int Longitude_before, Longitude_after;

uint8_t HeartRate = 0;
float ax, ay, az;
uint8_t Data[86];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Football_Player */
static void Custom_Pag_Update_Char(void);
static void Custom_Pag_Send_Notification(void);

/* USER CODE BEGIN PFP */

static void PAGMeas();
static void PAGAPP_Measurment();
static uint32_t PAGAPP_Read_RTC_SSR_SS();
void get_location();
void Max30102Loop();
extern void getAccMPU();

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* Football_Player */
    case CUSTOM_STM_PAG_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PAG_NOTIFY_ENABLED_EVT */
    	HW_TS_Stop(Custom_App_Context.TimerMeasurement_Id);
    	HW_TS_Start(Custom_App_Context.TimerMeasurement_Id, PAGAPP_MEASUREMENT_INTERVAL);
      /* USER CODE END CUSTOM_STM_PAG_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_PAG_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PAG_NOTIFY_DISABLED_EVT */
    	HW_TS_Stop(Custom_App_Context.TimerMeasurement_Id);
      /* USER CODE END CUSTOM_STM_PAG_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
//        APP_DBG_MSG("\r\ n \ r** CUSTOM_STM_PAG_NOTIFY_ENABLED_EVT \ n");

        Custom_App_Context.Pag_Notification_Status = 1;
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
//        APP_DBG_MSG("\ r\n \ r** CUSTOM_STM_PAG_NOTIFY_DISABLED_EVT \n");

        Custom_App_Context.Pag_Notification_Status = 0;
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
UTIL_SEQ_RegTask(1 << CFG_TASK_MEAS_REQ_ID, UTIL_SEQ_RFU, PAGAPP_Measurment);

Custom_App_Context.Pag_Notification_Status = 0;
Custom_App_Context.Data = 0;

HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(Custom_App_Context.TimerMeasurement_Id), hw_ts_Repeated, PAGMeas);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* Football_Player */
void Custom_Pag_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pag_UC_1*/
//
  /* USER CODE END Pag_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PAG, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Pag_UC_Last*/
//
  /* USER CODE END Pag_UC_Last*/
  return;
}

void Custom_Pag_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Pag_NS_1*/
//
  /* USER CODE END Pag_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_PAG, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Pag_NS_Last*/
//
  /* USER CODE END Pag_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

static void PAGAPP_Measurment()
{
	  getAccMPU();
	  get_location();
	  Max30102Loop();
	  snprintf(Data, (size_t)86, "a11adf7a-f226-435f-891d-5fbd5eae7445;%3d.%5d;%3d.%5d;%2d.%2d;%2d.%2d;%2d.%2d;%3d", //
			  Latitude_before,Latitude_after, Longitude_before, Longitude_after,
			  (uint8_t)ax, (uint8_t)(ax*100)%100,(uint8_t)ay, (uint8_t)(ay*100)%100, (uint8_t)az, (uint8_t)(az*100)%100,
			  HeartRate);
	  Custom_App_Context.Data = Data;
	  Custom_STM_App_Update_Char(CUSTOM_STM_PAG, (uint8_t*)Custom_App_Context.Data);

	  return;
}

static void PAGMeas()
{
	UTIL_SEQ_SetTask(1 << CFG_TASK_MEAS_REQ_ID, CFG_SCH_PRIO_0);
}

static uint32_t PAGAPP_Read_RTC_SSR_SS()
{
	return((uint32_t)(READ_BIT(RTC->SSR, RTC_SSR_SS)));
}

void getAccMPU()
{
	  while (updateMPU(&mpu)!=1);
	  ax = getAccX(&mpu);
	  ay = getAccY(&mpu);
	  az = getAccZ(&mpu);
}
void get_location()
{
	Msgindex = 0;
	strcpy(Txdata, (char*)(Rxdata));
	ptr = strstr(Txdata, "GPRMC");
	if(*ptr == 71)
	{
		while(1)
		{
			GPS_Payyload[Msgindex] = *ptr;
			Msgindex++;
			*ptr=*(ptr+Msgindex);
			if (*ptr == '\n' || *ptr == '\0')
			{
				GPS_Payyload[Msgindex] = "\0";
				break;
			}
		}
		if (*ptr == '\n'){
			sscanf(GPS_Payyload, "GPRMC,%d.%d,A,%d.%d,N,%d.%d,", &Time_before, &Time_before,
					&Latitude_before,&Latitude_after,
					&Longitude_before, &Longitude_after);
		}
	}
}

void Max30102Loop()
{
  char buf[20];
  float n_spo2;
  float ratio;
  float correl;
  int8_t ch_spo2_valid;  				// indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; 				// heart rate value
  int8_t  ch_hr_valid;  				// indicator to show if the heart rate calculation is valid
  uint8_t i;

  // buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  for (i = 0U; i < BUFFER_SIZE; i++)
  {
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET);	// wait until the interrupt pin asserts

#ifdef FAKE_MAX30102_CLONE
    maxim_max30102_read_fifo((aun_ir_buffer + i), (aun_red_buffer + i));  // read from MAX30102 FIFO
#else
    maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));  // read from MAX30102 FIFO
#endif
  }
  // calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer,
		  &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
  if (ch_hr_valid && ch_spo2_valid)
  {
    HeartRate = n_heart_rate;
  }
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
