/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "hw_if.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "sys_conf.h"
#include "gatt_service.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa endNode send request
  * @param  none
  * @retval none
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  timer context
  * @retval none
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  LED Tx timer callback function
  * @param  LED context
  * @retval none
  */
static void OnTxTimerLedEvent(void *context);

/**
  * @brief  LED Rx timer callback function
  * @param  LED context
  * @retval none
  */
static void OnRxTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  LED context
  * @retval none
  */
static void OnJoinTimerLedEvent(void *context);

/**
  * @brief  join event callback function
  * @param  params
  * @retval none
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief  tx event callback function
  * @param  params
  * @retval none
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRa endNode has received a frame
  * @param appData
  * @param params
  * @retval None
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/*!
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */
static void OnMacProcessNotify(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

static uint8_t CurrentAppData[16] = "STM32WB55 here!";
static uint16_t CurrentDataLength = 15;

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

static uint16_t AppTxDutyCycle = APP_TX_DUTYCYCLE;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =           NULL,
  .GetTemperature =            NULL,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};

/**
  * @brief Specifies the state of the application LED
  */
static uint8_t AppLedStateOn = RESET;

/**
  * @brief Timer to handle the application Tx
  */
static uint8_t TxTimer;

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static uint8_t TxLedTimer;

/**
  * @brief Timer to handle the application Rx Led to toggle
  */
static uint8_t RxLedTimer;

/**
  * @brief Timer to handle the application Join Led to toggle
  */
static uint8_t JoinLedTimer;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* USER CODE END LoRaWAN_Init_1 */
  LED_Init(LED_BLUE); // Join, blinks until JoinAccept received
  LED_Init(LED_RED); // Tx, lit 500ms after transmission
  LED_Init(LED_GREEN); // Rx, lit 500ms after reception

  /* Get LoRa APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION:        V%X.%X.%X\r\n",
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW LoraWAN info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:   V%X.%X.%X\r\n",
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB2_SHIFT));

  HW_TS_Create(1, &TxLedTimer, hw_ts_SingleShot, (HW_TS_pTimerCb_t)OnTxTimerLedEvent);
  HW_TS_Create(1, &RxLedTimer, hw_ts_SingleShot, (HW_TS_pTimerCb_t)OnRxTimerLedEvent);
  HW_TS_Create(1, &JoinLedTimer, hw_ts_Repeated, (HW_TS_pTimerCb_t)OnJoinTimerLedEvent);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks);

  LmHandlerConfigure(&LmHandlerParams);

  HW_TS_Start(JoinLedTimer, 500*1000/CFG_TS_TICK_VAL);

  LmHandlerJoin(ActivationType);

  /* send every time timer elapses */
  HW_TS_Create(1, &TxTimer, hw_ts_SingleShot, (HW_TS_pTimerCb_t)OnTxTimerEvent);
  HW_TS_Start(TxTimer, APP_TX_DUTYCYCLE*1000/CFG_TS_TICK_VAL);

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */

  /* USER CODE END OnRxData_1 */
  if ((appData != NULL) && (params != NULL))
  {
    LED_On(LED_GREEN);

#if defined (APP_LOG_ENABLED) && (APP_LOG_ENABLED == 1)
    static const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };

    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Indication ==========\r\n");
    APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\r\n",
            params->DownlinkCounter, slotStrings[params->RxSlot], appData->Port, params->Datarate, params->Rssi, params->Snr);
#endif

    updateRSSI(params->Rssi);
    updateSnr(params->Snr);

    switch (appData->Port)
    {
      case LORAWAN_SWITCH_CLASS_PORT:
        /*this port switches the class*/
        if (appData->BufferSize == 1)
        {
          switch (appData->Buffer[0])
          {
            case 0:
            {
              LmHandlerRequestClass(CLASS_A);
              break;
            }
            case 1:
            {
              LmHandlerRequestClass(CLASS_B);
              break;
            }
            case 2:
            {
              LmHandlerRequestClass(CLASS_C);
              break;
            }
            default:
              break;
          }
        }
        break;
      case LORAWAN_USER_APP_PORT:
        if (appData->BufferSize == 1)
        {
          AppLedStateOn = appData->Buffer[0] & 0x01;
          if (AppLedStateOn == RESET)
          {
            APP_LOG(TS_OFF, VLEVEL_H,   "LED OFF\r\n");

            LED_Off(LED_GREEN);
          }
          else
          {
            APP_LOG(TS_OFF, VLEVEL_H, "LED ON\r\n");

            LED_On(LED_GREEN);
          }
        }
        break;
      /* USER CODE BEGIN OnRxData_Switch_case */

      /* USER CODE END OnRxData_Switch_case */
      default:
        /* USER CODE BEGIN OnRxData_Switch_default */

        /* USER CODE END OnRxData_Switch_default */
        break;
    }
  }

  /* USER CODE BEGIN OnRxData_2 */

  /* USER CODE END OnRxData_2 */
}

void setLoraData(uint8_t *data, uint16_t length) {
	  memcpy(CurrentAppData, data, length);
	  CurrentDataLength = length;
}

void setLoraPeriod(uint16_t period) {
	AppTxDutyCycle = period*1000;
}

static void SendTxData(void)
{
  uint32_t nextTxIn = 0;
  /* USER CODE BEGIN SendTxData_1 */

  memcpy(AppData.Buffer, CurrentAppData, CurrentDataLength);
  AppData.BufferSize = CurrentDataLength;

  /* USER CODE END SendTxData_1 */

  AppData.Port = LORAWAN_USER_APP_PORT;

  if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
  {
    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
  }
  else if (nextTxIn > 0)
  {
    APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
  }

  /*Wait for next tx slot*/
  HW_TS_Start(TxTimer, AppTxDutyCycle*1000/CFG_TS_TICK_VAL);

  // Update GATT status characteristic
  updateStatus(LmHandlerJoinStatus());

  /* USER CODE BEGIN SendTxData_2 */

  /* USER CODE END SendTxData_2 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SCH_PRIO_0);

  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

static void OnTxTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerLedEvent_1 */

  /* USER CODE END OnTxTimerLedEvent_1 */
  LED_Off(LED_RED);
  /* USER CODE BEGIN OnTxTimerLedEvent_2 */

  /* USER CODE END OnTxTimerLedEvent_2 */
}

static void OnRxTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnRxTimerLedEvent_1 */

  /* USER CODE END OnRxTimerLedEvent_1 */
  LED_Off(LED_GREEN) ;
  /* USER CODE BEGIN OnRxTimerLedEvent_2 */

  /* USER CODE END OnRxTimerLedEvent_2 */
}

static void OnJoinTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnJoinTimerLedEvent_1 */

  /* USER CODE END OnJoinTimerLedEvent_1 */
  LED_Toggle(LED_BLUE) ;
  /* USER CODE BEGIN OnJoinTimerLedEvent_2 */

  /* USER CODE END OnJoinTimerLedEvent_2 */
}

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */

  /* USER CODE END OnTxData_1 */
  if ((params != NULL) && (params->IsMcpsConfirm != 0))
  {
    LED_On(LED_RED) ;

    HW_TS_Start(TxLedTimer, 500*1000/CFG_TS_TICK_VAL);

    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
    APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
            params->AppData.Port, params->Datarate, params->TxPower);

    APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
    if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
    {
      APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
    }
  }

  /* USER CODE BEGIN OnTxData_2 */

  /* USER CODE END OnTxData_2 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */

  /* USER CODE END OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
      HW_TS_Stop(JoinLedTimer);

      LED_Off(LED_BLUE) ;

      updateStatus(LmHandlerJoinStatus());

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
      }
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }
  }

  /* USER CODE BEGIN OnJoinRequest_2 */

  /* USER CODE END OnJoinRequest_2 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SCH_PRIO_0);
  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
