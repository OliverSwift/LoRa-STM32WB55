/**
******************************************************************************
* @file    gatt_service.c
* @author  O.Debon
* @brief   BLE Lora GATT Server
******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ble.h"
#include "gatt_service.h"
#include "secure-element.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t LoraServiceHandle;
  uint16_t StatusCharacteristicHandle;
  uint16_t DevEUICharacteristicHandle;
  uint16_t JoinEUICharacteristicHandle;
  uint16_t DataCharacteristicHandle;
  uint16_t PeriodCharacteristicHandle;
  uint16_t RSSICharacteristicHandle;
  uint16_t SnrCharacteristicHandle;
} LoraServiceContext_t;

/* Private defines -----------------------------------------------------------*/
/* My Very Own Service and Characteristics UUIDs */

#define COPY_LORA_SERVICE_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x00,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_STATUS_CHARACTERISTIC_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x01,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_DEVEUI_CHARACTERISTIC_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x02,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_JOINEUI_CHARACTERISTIC_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x03,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_DATA_CHARACTERISTIC_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x04,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_PERIOD_CHARACTERISTIC_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x05,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_RSSI_CHARACTERISTIC_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x06,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_SNR_CHARACTERISTIC_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x07,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)

/** Max_Attribute_Records = 2*no_of_char + 1
  * service_max_attribute_record = 1 for My Very Own service +
  *                                2 for My Very Own Read characteristic +
  *                                
  */
#define MY_VERY_OWN_SERVICE_MAX_ATT_RECORDS                (1+7*2)

#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1

/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
  uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
      uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
        uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Private variables ---------------------------------------------------------*/

/**
* START of Section BLE_DRIVER_CONTEXT
*/
PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static LoraServiceContext_t loraServiceContext;

/**
* END of Section BLE_DRIVER_CONTEXT
*/
/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t LoraService_EventHandler(void *pckt);

/* Functions Definition ------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Event handler
* @param  Event: Address of the buffer holding the Event
* @retval Ack: Return whether the Event has been managed or not
*/
static SVCCTL_EvtAckStatus_t LoraService_EventHandler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);
  
#define CHAR_VALUE_HANDLE_OFFSET 1
#define CHAR_DESCRIPTOR_HANDLE_OFFSET 2

  switch(event_pckt->evt)
  {
  case EVT_VENDOR:
    {
      blue_evt = (evt_blue_aci*)event_pckt->data;
      switch(blue_evt->ecode)
      {
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blue_evt->data;
        if(attribute_modified->Attr_Handle == (loraServiceContext.PeriodCharacteristicHandle + CHAR_VALUE_HANDLE_OFFSET)
        		&& attribute_modified->Attr_Data_Length == 2) {
        	uint16_t seconds;
			seconds = *((uint16_t *)attribute_modified->Attr_Data);
        	setLoraPeriod(seconds);
        	return_value = SVCCTL_EvtAckFlowEnable;
        } else if(attribute_modified->Attr_Handle == (loraServiceContext.DataCharacteristicHandle + CHAR_VALUE_HANDLE_OFFSET)) {
			setLoraData(attribute_modified->Attr_Data, attribute_modified->Attr_Data_Length);
			return_value = SVCCTL_EvtAckFlowEnable;
		}
        break;
        
      default:
        break;
      }
    }
    break; /* HCI_EVT_VENDOR_SPECIFIC */
    
  default:
    break;
  }
  
  return(return_value);
}/* end SVCCTL_EvtAckStatus_t */

/* Public functions ----------------------------------------------------------*/

/**
* @brief  Service initialization
* @param  None
* @retval None
*/
void LoraService_Init(void)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;
  Char_UUID_t  uuid16;
  
  /**
  *	Register the event handler to the BLE controller
  */
  SVCCTL_RegisterSvcHandler(LoraService_EventHandler);
  
  /**
  *  Add LEDisplay Service
  */
  COPY_LORA_SERVICE_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                       (Service_UUID_t *) &uuid16,
                       PRIMARY_SERVICE,
                       MY_VERY_OWN_SERVICE_MAX_ATT_RECORDS,
                       &(loraServiceContext.LoraServiceHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }
  
  /**
  *  Add Status Characteristic
  */
  COPY_STATUS_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(loraServiceContext.LoraServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    1,
                    CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
					GATT_DONT_NOTIFY_EVENTS, /* gattEvtMask */
                    10, /* encryKeySize */
                    0, /* isVariable */
                    &(loraServiceContext.StatusCharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

  /**
  *  Add DevEUI Characteristic
  */
  COPY_DEVEUI_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(loraServiceContext.LoraServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    8, /* 8 * uint8_t */
                    CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
					GATT_DONT_NOTIFY_EVENTS, /* gattEvtMask */
                    10, /* encryKeySize */
                    0, /* isVariable */
                    &(loraServiceContext.DevEUICharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

  /**
  *  Add JoinEUI Characteristic
  */
  COPY_JOINEUI_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(loraServiceContext.LoraServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    8, /* 8 * uint8_t */
                    CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
					GATT_DONT_NOTIFY_EVENTS, /* gattEvtMask */
                    10, /* encryKeySize */
                    0, /* isVariable */
                    &(loraServiceContext.JoinEUICharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

  /**
  *  Add Data Characteristic
  */
  COPY_DATA_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(loraServiceContext.LoraServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    16, /* Max 16 bytes */
                    CHAR_PROP_WRITE,
                    ATTR_PERMISSION_NONE,
					GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                    10, /* encryKeySize */
                    1, /* isVariable */
                    &(loraServiceContext.DataCharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

  /**
  *  Add Period Characteristic
  */
  COPY_PERIOD_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(loraServiceContext.LoraServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    2, /* uint16_t */
                    CHAR_PROP_WRITE,
                    ATTR_PERMISSION_NONE,
					GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                    10, /* encryKeySize */
                    0, /* isVariable */
                    &(loraServiceContext.PeriodCharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

  /**
  *  Add RSSI Characteristic
  */
  COPY_RSSI_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(loraServiceContext.LoraServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    1,
                    CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
					GATT_DONT_NOTIFY_EVENTS, /* gattEvtMask */
                    10, /* encryKeySize */
                    0, /* isVariable */
                    &(loraServiceContext.RSSICharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

  /**
  *  Add SNR Characteristic
  */
  COPY_SNR_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(loraServiceContext.LoraServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    1,
                    CHAR_PROP_READ,
                    ATTR_PERMISSION_NONE,
					GATT_DONT_NOTIFY_EVENTS, /* gattEvtMask */
                    10, /* encryKeySize */
                    0, /* isVariable */
                    &(loraServiceContext.SnrCharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

  /*
   * Initializations
   */
  uint8_t *eui;

  eui = SecureElementGetJoinEui();
  aci_gatt_update_char_value(loraServiceContext.LoraServiceHandle,
                                      loraServiceContext.JoinEUICharacteristicHandle,
                                      0, /* charValOffset */
                                      8, /* charValueLen */
                                      (uint8_t *) eui);

  eui = SecureElementGetDevEui();
  aci_gatt_update_char_value(loraServiceContext.LoraServiceHandle,
                                      loraServiceContext.DevEUICharacteristicHandle,
                                      0, /* charValOffset */
                                      8, /* charValueLen */
                                      (uint8_t *) eui);


  return;
} /* MyVeryOwnService_Init() */

void updateStatus(uint8_t status) {
	  tBleStatus ret = BLE_STATUS_SUCCESS;

	  /* Initialize Duration */
	  ret = aci_gatt_update_char_value(loraServiceContext.LoraServiceHandle,
	                                      loraServiceContext.StatusCharacteristicHandle,
	                                      0, /* charValOffset */
	                                      1, /* charValueLen */
	                                      (uint8_t *) &status);
	  if (ret != BLE_STATUS_SUCCESS)
	  {
	    Error_Handler(); /* UNEXPECTED */
	  }
}

void updateDevEUI(uint8_t *deveui) {
	  tBleStatus ret = BLE_STATUS_SUCCESS;

	  /* Initialize Duration */
	  ret = aci_gatt_update_char_value(loraServiceContext.LoraServiceHandle,
	                                      loraServiceContext.DevEUICharacteristicHandle,
	                                      0, /* charValOffset */
	                                      8, /* charValueLen */
	                                      (uint8_t *) deveui);
	  if (ret != BLE_STATUS_SUCCESS)
	  {
	    Error_Handler(); /* UNEXPECTED */
	  }

}

void updateJoinEUI(uint8_t *joineui) {
	  tBleStatus ret = BLE_STATUS_SUCCESS;

	  /* Initialize Duration */
	  ret = aci_gatt_update_char_value(loraServiceContext.LoraServiceHandle,
	                                      loraServiceContext.JoinEUICharacteristicHandle,
	                                      0, /* charValOffset */
	                                      8, /* charValueLen */
	                                      (uint8_t *) joineui);
	  if (ret != BLE_STATUS_SUCCESS)
	  {
	    Error_Handler(); /* UNEXPECTED */
	  }

}

void updateRSSI(uint8_t rssi) {
	  tBleStatus ret = BLE_STATUS_SUCCESS;

	  /* Initialize Duration */
	  ret = aci_gatt_update_char_value(loraServiceContext.LoraServiceHandle,
	                                      loraServiceContext.RSSICharacteristicHandle,
	                                      0, /* charValOffset */
	                                      1, /* charValueLen */
	                                      (uint8_t *) &rssi);
	  if (ret != BLE_STATUS_SUCCESS)
	  {
	    Error_Handler(); /* UNEXPECTED */
	  }

}

void updateSnr(uint8_t snr) {
	  tBleStatus ret = BLE_STATUS_SUCCESS;

	  /* Initialize Duration */
	  ret = aci_gatt_update_char_value(loraServiceContext.LoraServiceHandle,
	                                      loraServiceContext.SnrCharacteristicHandle,
	                                      0, /* charValOffset */
	                                      1, /* charValueLen */
	                                      (uint8_t *) &snr);
	  if (ret != BLE_STATUS_SUCCESS)
	  {
	    Error_Handler(); /* UNEXPECTED */
	  }

}

/*
 * Overrides the SVCCTL_SvcInit from svc_ctl.c in Middlewares/ST/STM32_WPAN/ble/svc
 * which is called by SVCCTL_Init called by APP_BLE_Init (app_ble.c)
 */

void SVCCTL_SvcInit(void) {
	  DIS_Init();
	  LoraService_Init();
}

