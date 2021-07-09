/**
******************************************************************************
* @file    gatt_service.c
* @author  O.Debon
* @brief   BLE Test GATT Server
******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ble.h"
#include "gatt_service.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t TestServiceHandle;
  uint16_t LedStatusCharacteristicHandle;
} TestServiceContext_t;

/* Private defines -----------------------------------------------------------*/
/* My Very Own Service and Characteristics UUIDs */

#define COPY_TEST_SERVICE_UUID(uuid_struct)                COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x00,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)
#define COPY_LEDSTATUS_CHARACTERISTIC_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0xcc,0x26,0x00,0x01,0xcf,0x94,0x4f,0xe8,0xb2,0x7c,0xce,0x7a,0x4d,0xc8,0x94,0x8d)

/** Max_Attribute_Records = 2*no_of_char + 1
  * service_max_attribute_record = 1 for My Very Own service +
  *                                2 for My Very Own Read characteristic +
  *                                
  */
#define MY_VERY_OWN_SERVICE_MAX_ATT_RECORDS                3

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
PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static TestServiceContext_t testServiceContext;

/**
* END of Section BLE_DRIVER_CONTEXT
*/
/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t TestService_EventHandler(void *pckt);

/* Functions Definition ------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Event handler
* @param  Event: Address of the buffer holding the Event
* @retval Ack: Return whether the Event has been managed or not
*/
static SVCCTL_EvtAckStatus_t TestService_EventHandler(void *Event)
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
        if(attribute_modified->Attr_Handle == (testServiceContext.LedStatusCharacteristicHandle + CHAR_VALUE_HANDLE_OFFSET))
        {
        	if (attribute_modified->Attr_Data[0]) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        	} else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        	}
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
void TestService_Init(void)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;
  Char_UUID_t  uuid16;
  
  /**
  *	Register the event handler to the BLE controller
  */
  SVCCTL_RegisterSvcHandler(TestService_EventHandler);
  
  /**
  *  Add LEDisplay Service
  */
  COPY_TEST_SERVICE_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                       (Service_UUID_t *) &uuid16,
                       PRIMARY_SERVICE,
                       MY_VERY_OWN_SERVICE_MAX_ATT_RECORDS,
                       &(testServiceContext.TestServiceHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }
  
  /**
  *  Add Led Status Characteristic
  */
  COPY_LEDSTATUS_CHARACTERISTIC_UUID(uuid16.Char_UUID_128);
  ret = aci_gatt_add_char(testServiceContext.TestServiceHandle,
                    UUID_TYPE_128, &uuid16,
                    19,
                    CHAR_PROP_WRITE,
                    ATTR_PERMISSION_NONE,
                    GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                    10, /* encryKeySize */
                    1, /* isVariable */
                    &(testServiceContext.LedStatusCharacteristicHandle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }

#if 0
  /* Initialize Duration */
  ret = aci_gatt_update_char_value(leDisplayServiceContext.LeDisplayServiceHandle,
                                      leDisplayServiceContext.DurationCharacteristicHandle,
                                      0, /* charValOffset */
                                      1, /* charValueLen */
                                      (uint8_t *) &duration);
  if (ret != BLE_STATUS_SUCCESS)
  {
    Error_Handler(); /* UNEXPECTED */
  }
#endif

  return;
} /* MyVeryOwnService_Init() */

/*
 * Overrides the SVCCTL_SvcInit from svc_ctl.c in Middlewares/ST/STM32_WPAN/ble/svc
 * which is called by SVCCTL_Init called by APP_BLE_Init (app_ble.c)
 */

void SVCCTL_SvcInit(void) {
	  DIS_Init();
	  TestService_Init();
}

