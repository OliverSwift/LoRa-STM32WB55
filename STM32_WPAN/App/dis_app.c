/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : App/dis_app.c
  * Description        : Device Information Service Application
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
#include "dis_app.h"

/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if ((BLE_CFG_DIS_SYSTEM_ID != 0) || (CFG_MENU_DEVICE_INFORMATION != 0))
static const uint8_t system_id[BLE_CFG_DIS_SYSTEM_ID_LEN_MAX] =
{
  (uint8_t)((DISAPP_MANUFACTURER_ID & 0xFF0000) >> 16),
  (uint8_t)((DISAPP_MANUFACTURER_ID & 0x00FF00) >> 8),
  (uint8_t)(DISAPP_MANUFACTURER_ID & 0x0000FF),
  0xFE,
  0xFF,
  (uint8_t)((DISAPP_OUI & 0xFF0000) >> 16),
  (uint8_t)((DISAPP_OUI & 0x00FF00) >> 8),
  (uint8_t)(DISAPP_OUI & 0x0000FF)
};
#endif

#if ((BLE_CFG_DIS_IEEE_CERTIFICATION != 0) || (CFG_MENU_DEVICE_INFORMATION != 0))
static const uint8_t ieee_id[BLE_CFG_DIS_IEEE_CERTIFICATION_LEN_MAX] =
{
  0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA,
  0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA,
  0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA,
  0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA,
};
#endif
#if ((BLE_CFG_DIS_PNP_ID != 0) || (CFG_MENU_DEVICE_INFORMATION != 0))
static const uint8_t pnp_id[BLE_CFG_DIS_PNP_ID_LEN_MAX] =
{
  0x1,
  0xAD, 0xDE,
  0xDE, 0xDA,
  0x01, 0x00
};
#endif
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void DISAPP_Init(void)
{
/* USER CODE BEGIN DISAPP_Init */

/* USER CODE END DISAPP_Init */
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
