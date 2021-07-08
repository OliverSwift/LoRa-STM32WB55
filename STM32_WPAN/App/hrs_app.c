/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : App/hrs_app.c
  * Description        : Heart Rate Service Application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"

#include "ble.h"
#include "hrs_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  HRS_BodySensorLocation_t BodySensorLocationChar;
  HRS_MeasVal_t MeasurementvalueChar;
  uint8_t ResetEnergyExpended;
  uint8_t TimerMeasurement_Id;

} HRSAPP_Context_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define HRSAPP_MEASUREMENT_INTERVAL   (1000000/CFG_TS_TICK_VAL)  /**< 1s */
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static HRSAPP_Context_t HRSAPP_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions prototypes-----------------------------------------------*/
static void HrMeas( void );
static void HRSAPP_Measurement(void);
static uint32_t HRSAPP_Read_RTC_SSR_SS ( void );
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void HRS_Notification(HRS_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN HRS_Notification_1 */

/* USER CODE END HRS_Notification_1 */
  switch(pNotification->HRS_Evt_Opcode)
  {
/* USER CODE BEGIN HRS_Notification_HRS_Evt_Opcode */

/* USER CODE END HRS_Notification_HRS_Evt_Opcode */
#if (BLE_CFG_HRS_ENERGY_EXPENDED_INFO_FLAG != 0)
    case HRS_RESET_ENERGY_EXPENDED_EVT:
/* USER CODE BEGIN HRS_RESET_ENERGY_EXPENDED_EVT */

/* USER CODE END HRS_RESET_ENERGY_EXPENDED_EVT */
      break;
#endif

    case HRS_NOTIFICATION_ENABLED:
/* USER CODE BEGIN HRS_NOTIFICATION_ENABLED */

/* USER CODE END HRS_NOTIFICATION_ENABLED */
      break;

    case HRS_NOTIFICATION_DISABLED:
/* USER CODE BEGIN HRS_NOTIFICATION_DISABLED */

/* USER CODE END HRS_NOTIFICATION_DISABLED */
      break;

#if (BLE_CFG_OTA_REBOOT_CHAR != 0)
    case HRS_STM_BOOT_REQUEST_EVT:
/* USER CODE BEGIN HRS_STM_BOOT_REQUEST_EVT */

/* USER CODE END HRS_STM_BOOT_REQUEST_EVT */
      break;
#endif

   default:
/* USER CODE BEGIN HRS_Notification_Default */

/* USER CODE END HRS_Notification_Default */
      break;
  }
/* USER CODE BEGIN HRS_Notification_2 */

/* USER CODE END HRS_Notification_2 */
  return;
}

void HRSAPP_Init(void)
{
  UTIL_SEQ_RegTask( 1<< CFG_TASK_MEAS_REQ_ID, UTIL_SEQ_RFU, HRSAPP_Measurement );
/* USER CODE BEGIN HRSAPP_Init */

/* USER CODE END HRSAPP_Init */
  return;
}

static void HRSAPP_Measurement(void)
{
/* USER CODE BEGIN HRSAPP_Measurement */

/* USER CODE END HRSAPP_Measurement */
  return;
}

static void HrMeas( void )
{
  /**
   * The code shall be executed in the background as aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask( 1<<CFG_TASK_MEAS_REQ_ID, CFG_SCH_PRIO_0);
/* USER CODE BEGIN HrMeas */

/* USER CODE END HrMeas */

  return;
}

static uint32_t HRSAPP_Read_RTC_SSR_SS ( void )
{
  return ((uint32_t)(READ_BIT(RTC->SSR, RTC_SSR_SS)));
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
