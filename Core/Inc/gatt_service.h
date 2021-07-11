/**
  ******************************************************************************
  * @file    gatt_service.h
  * @author  MCD Application Team
  * @brief   Header for gatt_service.c module
  ******************************************************************************
  * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GATT_SERVICE_H
#define __GATT_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void LoraService_Init(void);

void updateRole(uint8_t master);
void updateCounter(uint16_t counter);
void updateRSSI(uint8_t rssi);
void updateSnr(uint8_t snr);

#ifdef __cplusplus
}
#endif

#endif /* __GATT_SERVICE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
