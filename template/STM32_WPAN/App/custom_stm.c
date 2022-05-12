/* USER CODE BEGIN Header */
/**
 * @file custom_stm.c
 * @brief Custom Server Initialization.
 */
/*
  ******************************************************************************
  * File Name          : App/custom_stm.c
  * Description        : Custom Example Service.
  *
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
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint16_t CustomClock_SHdle;
  uint16_t CustomClock_Datetime_CHdle;
  uint16_t CustomAlarm1_SHdle;
  uint16_t CustomAlarm1_Time_CHdle;
  uint16_t CustomAlarm1_Status_CHdle;
  uint16_t CustomAlarm2_SHdle;
  uint16_t CustomAlarm2_Time_CHdle;
  uint16_t CustomAlarm2_Status_CHdle;
  uint16_t CustomAlarm3_SHdle;
  uint16_t CustomAlarm3_Time_CHdle;
  uint16_t CustomAlarm3_Status_CHdle;
  uint16_t CustomAlarm4_SHdle;
  uint16_t CustomAlarm4_Time_CHdle;
  uint16_t CustomAlarm4_Status_CHdle;
  uint16_t CustomAlarm5_SHdle;
  uint16_t CustomAlarm5_Time_CHdle;
  uint16_t CustomAlarm5_Status_CHdle;
  uint16_t CustomAlarm6_SHdle;
  uint16_t CustomAlarm6_Time_CHdle;
  uint16_t CustomAlarm6_Status_CHdle;
  uint16_t CustomAlarm7_SHdle;
  uint16_t CustomAlarm7_Time_CHdle;
  uint16_t CustomAlarm7_Status_CHdle;
  uint16_t CustomAlarm8_SHdle;
  uint16_t CustomAlarm8_Time_CHdle;
  uint16_t CustomAlarm8_Status_CHdle;
  uint16_t CustomAlarm9_SHdle;
  uint16_t CustomAlarm9_Time_CHdle;
  uint16_t CustomAlarm9_Status_CHdle;
  uint16_t CustomAlarm10_SHdle;
  uint16_t CustomAlarm10_Time_CHdle;
  uint16_t CustomAlarm10_Status_CHdle;
  uint16_t CustomStopwatch_SHdle;
  uint16_t CustomStopwatch_Time_S_CHdle;
  uint16_t CustomStopwatch_Status_CHdle;
  uint16_t CustomTimer_SHdle;
  uint16_t CustomTimer_Time_S_CHdle;
  uint16_t CustomTimer_Set_Time_S_CHdle;
  uint16_t CustomTimer_Status_CHdle;
  uint16_t CustomSettings_SHdle;
  uint16_t CustomSettings_Led_Blink_Interval_Ms_CHdle;
  uint16_t CustomSettings_Time_Display_Duration_Ms_CHdle;
  uint16_t CustomSettings_Date_Display_Duration_Ms_CHdle;
  uint16_t CustomSettings_Alarm_Ringing_Duration_Ms_CHdle;
  uint16_t CustomSettings_Timer_Ringing_Duration_Ms_CHdle;
  uint16_t CustomSettings_Date_Format_CHdle;
  uint16_t CustomRtcCalibration_SHdle;
  uint16_t CustomRtcCalibration_Calp_CHdle;
  uint16_t CustomRtcCalibration_Calm_CHdle;
} CustomContext_t;

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED (1)

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         (2)
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              (1)
/* USER CODE BEGIN PM */
#define DATETIME_UUID (0x2a08)
#define CHAR_ENC_KEY_SIZE (0x10)
// Max_Attribute_Records = 1 + 2*no_of_char + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
#define CLOCK_SERVICE_MAX_ATTR_RECORDS (4)
#define ALARM_SERVICE_MAX_ATTR_RECORDS (5)
#define STOPWATCH_SERVICE_MAX_ATTR_RECORDS (7)
#define TIMER_SERVICE_MAX_ATTR_RECORDS (9)
#define SETTINGS_SERVICE_MAX_ATTR_RECORDS (13)
#define RTC_CALIBRATION_SERVICE_MAX_ATTR_RECORDS (5)
#define CLOCK_DATETIME_VALUE_OFFSET (0)
#define ALARM_TIME_VALUE_OFFSET (0)
#define ALARM_STATUS_VALUE_OFFSET (0)
#define STOPWATCH_TIME_VALUE_OFFSET (0)
#define STOPWATCH_STATUS_VALUE_OFFSET (0)
#define TIMER_TIME_VALUE_OFFSET (0)
#define TIMER_SET_TIME_VALUE_OFFSET (0)
#define TIMER_STATUS_VALUE_OFFSET (0)
#define SETTINGS_DURATION_VALUE_OFFSET (0)
#define SETTINGS_DATE_FORMAT_VALUE_OFFSET (0)
#define RTC_CALIBRATION_CALP_VALUE_OFFSET (0)
#define RTC_CALIBRATION_CALM_VALUE_OFFSET (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/*
 * START of Section BLE_DRIVER_CONTEXT
 */

PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static CustomContext_t CustomContext;

/*
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */
/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
#define COPY_CLOCK_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x00,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)

#define COPY_ALARM1_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x10,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM2_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x20,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM3_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x30,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM4_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x40,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM5_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x50,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM6_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x60,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM7_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x70,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM8_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x80,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM9_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x90,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_ALARM10_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xa0,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)

#define COPY_ALARM_STATUS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0x11,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)

#define COPY_STOPWATCH_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xb0,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_STOPWATCH_TIME_S_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xb1,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_STOPWATCH_STATUS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xb2,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)

#define COPY_TIMER_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xc0,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_TIMER_TIME_S_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xc1,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_TIMER_SET_TIME_S_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xc2,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_TIMER_STATUS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xc3,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)

#define COPY_SETTINGS_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xd0,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_SETTINGS_LED_BLINK_INTERVAL_MS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xd1,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_SETTINGS_TIME_DISPLAY_DURATION_MS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xd2,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_SETTINGS_DATE_DISPLAY_DURATION_MS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xd3,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_SETTINGS_ALARM_RINGING_DURATION_MS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xd4,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_SETTINGS_TIMER_RINGING_DURATION_MS_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xd5,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_SETTINGS_DATE_FORMAT_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xd6,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)

#define COPY_RTC_CALIBRATION_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xe0,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_RTC_CALIBRATION_CALP_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xe1,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)
#define COPY_RTC_CALIBRATION_CALM_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x75,0xe2,0xf9,0x08,0x44,0xb0,0x81,0xb9,0x34,0x50,0xca,0x66,0x3f,0x46)

/* USER CODE BEGIN PF */
/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch(blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*) blecore_evt->data;

          //Clock
          if (attribute_modified->Attr_Handle == (CustomContext.CustomClock_Datetime_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_CLOCK_DATETIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomClock_Datetime_CHdle
              + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_CLOCK_DATETIME_NOTIFY_ENABLED_EVT;
            }
            else {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_CLOCK_DATETIME_NOTIFY_DISABLED_EVT;
            }
            Custom_STM_App_Notification(&Notification);
          }

          //Alarm1
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm1_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM1_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm1_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM1_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm2
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm2_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM2_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm2_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM2_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm3
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm3_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM3_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm3_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM3_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm4
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm4_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM4_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm4_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM4_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm5
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm5_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM5_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm5_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM5_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm6
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm6_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM6_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm6_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM6_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm7
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm7_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM7_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm7_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM7_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm8
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm8_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM8_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm8_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM8_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm9
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm9_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM9_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm9_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM9_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          //Alarm10
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm10_Time_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM10_TIME_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomAlarm10_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_ALARM10_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          // Stopwatch
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomStopwatch_Time_S_CHdle
              + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_STOPWATCH_TIME_S_NOTIFY_ENABLED_EVT;
            }
            else {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_STOPWATCH_TIME_S_NOTIFY_DISABLED_EVT;
            }
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomStopwatch_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_STOPWATCH_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }

          // Timer
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomTimer_Time_S_CHdle
              + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_TIMER_TIME_S_NOTIFY_ENABLED_EVT;
            }
            else {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_TIMER_TIME_S_NOTIFY_DISABLED_EVT;
            }
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomTimer_Set_Time_S_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_TIMER_SET_TIME_S_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomTimer_Status_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_TIMER_STATUS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          } 
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomTimer_Status_CHdle
              + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_TIMER_STATUS_NOTIFY_ENABLED_EVT;
            }
            else {
              Notification.Custom_Evt_Opcode = CUSTOM_STM_TIMER_STATUS_NOTIFY_DISABLED_EVT;
            }
            Custom_STM_App_Notification(&Notification);
          }     
          // Settings
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSettings_Led_Blink_Interval_Ms_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SETTINGS_LED_BLINK_INTERVAL_MS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSettings_Time_Display_Duration_Ms_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SETTINGS_TIME_DISPLAY_DURATION_MS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSettings_Date_Display_Duration_Ms_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SETTINGS_DATE_DISPLAY_DURATION_MS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSettings_Alarm_Ringing_Duration_Ms_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SETTINGS_ALARM_RINGING_DURATION_MS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSettings_Timer_Ringing_Duration_Ms_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SETTINGS_TIMER_RINGING_DURATION_MS_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomSettings_Date_Format_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_SETTINGS_DATE_FORMAT_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomRtcCalibration_Calp_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_RTC_CALIBRATION_CALP_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomRtcCalibration_Calm_CHdle
              + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)) {
            return_value = SVCCTL_EvtAckFlowEnable;
            Notification.Custom_Evt_Opcode = CUSTOM_STM_RTC_CALIBRATION_CALM_WRITE_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
          }
          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;
        /* USER CODE BEGIN BLECORE_EVT */
        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */
          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/
      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */
      /* USER CODE BEGIN EVENT_PCKT_CASES*/
      /* USER CODE END EVENT_PCKT_CASES*/
    default:
      /* USER CODE BEGIN EVENT_PCKT*/
      /* USER CODE END EVENT_PCKT*/
      break;
  }
  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */
  /* USER CODE END Custom_STM_Event_Handler_2 */
  return return_value;
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */
  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /*
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  COPY_CLOCK_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, CLOCK_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomClock_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomClock_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomClock_Datetime_CHdle));

  COPY_ALARM1_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm1_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm1_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm1_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm1_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm1_Status_CHdle));

  COPY_ALARM2_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm2_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm2_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm2_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm2_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm2_Status_CHdle));

  COPY_ALARM3_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm3_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm3_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm3_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm3_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm3_Status_CHdle));

  COPY_ALARM4_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm4_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm4_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm4_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm4_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm4_Status_CHdle));

  COPY_ALARM5_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm5_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm5_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm5_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm5_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm5_Status_CHdle));

  COPY_ALARM6_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm6_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm6_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm6_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm6_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm6_Status_CHdle));

  COPY_ALARM7_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm7_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm7_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm7_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm7_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm7_Status_CHdle));

  COPY_ALARM8_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm8_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm8_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm8_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm8_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm8_Status_CHdle));

  COPY_ALARM9_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm9_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm9_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm9_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm9_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm9_Status_CHdle));

  COPY_ALARM10_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, ALARM_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomAlarm10_SHdle));
  uuid.Char_UUID_16 = DATETIME_UUID;
  aci_gatt_add_char(CustomContext.CustomAlarm10_SHdle,
  UUID_TYPE_16, &uuid, CUSTOM_STM_DATETIME_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm10_Time_CHdle));
  COPY_ALARM_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomAlarm10_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomAlarm10_Status_CHdle));

  COPY_STOPWATCH_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, STOPWATCH_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomStopwatch_SHdle));
  COPY_STOPWATCH_TIME_S_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomStopwatch_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_NOTIFY,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomStopwatch_Time_S_CHdle));
  COPY_STOPWATCH_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomStopwatch_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomStopwatch_Status_CHdle));

  COPY_TIMER_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, TIMER_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomTimer_SHdle));
  COPY_TIMER_TIME_S_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomTimer_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_NOTIFY,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomTimer_Time_S_CHdle));
  COPY_TIMER_SET_TIME_S_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomTimer_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomTimer_Set_Time_S_CHdle));
  COPY_TIMER_STATUS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomTimer_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_STATUS_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomTimer_Status_CHdle));

  COPY_SETTINGS_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, SETTINGS_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomSettings_SHdle));
  COPY_SETTINGS_LED_BLINK_INTERVAL_MS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomSettings_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomSettings_Led_Blink_Interval_Ms_CHdle));
  COPY_SETTINGS_TIME_DISPLAY_DURATION_MS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomSettings_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomSettings_Time_Display_Duration_Ms_CHdle));
  COPY_SETTINGS_DATE_DISPLAY_DURATION_MS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomSettings_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomSettings_Date_Display_Duration_Ms_CHdle));
  COPY_SETTINGS_ALARM_RINGING_DURATION_MS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomSettings_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomSettings_Alarm_Ringing_Duration_Ms_CHdle));
  COPY_SETTINGS_TIMER_RINGING_DURATION_MS_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomSettings_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_UINT32_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomSettings_Timer_Ringing_Duration_Ms_CHdle));
  COPY_SETTINGS_DATE_FORMAT_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomSettings_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_DATE_FORMAT_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomSettings_Date_Format_CHdle));

  COPY_RTC_CALIBRATION_SERVICE_UUID(uuid.Char_UUID_128);
  aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*) &uuid,
  PRIMARY_SERVICE, RTC_CALIBRATION_SERVICE_MAX_ATTR_RECORDS, &(CustomContext.CustomRtcCalibration_SHdle));
  COPY_RTC_CALIBRATION_CALP_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomRtcCalibration_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_CALP_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomRtcCalibration_Calp_CHdle));
  COPY_RTC_CALIBRATION_CALM_CHAR_UUID(uuid.Char_UUID_128);
  aci_gatt_add_char(CustomContext.CustomRtcCalibration_SHdle,
  UUID_TYPE_128, &uuid, CUSTOM_STM_CALM_CHAR_SIZE,
  CHAR_PROP_READ | CHAR_PROP_WRITE,
  ATTR_PERMISSION_NONE,
  GATT_NOTIFY_ATTRIBUTE_WRITE, CHAR_ENC_KEY_SIZE,
  CHAR_VALUE_LEN_CONSTANT, &(CustomContext.CustomRtcCalibration_Calm_CHdle));
  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */
  /* USER CODE END SVCCTL_InitCustomSvc_2 */
  return;
}

/*
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t const *pPayload)
{
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */
  /* USER CODE END Custom_STM_App_Update_Char_1 */
  switch(CharOpcode)
  {
    case CUSTOM_STM_CLOCK_DATETIME:
      result = aci_gatt_update_char_value(CustomContext.CustomClock_SHdle,
          CustomContext.CustomClock_Datetime_CHdle, CLOCK_DATETIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM1_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm1_SHdle,
          CustomContext.CustomAlarm1_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM1_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm1_SHdle,
          CustomContext.CustomAlarm1_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM2_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm2_SHdle,
          CustomContext.CustomAlarm2_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM2_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm2_SHdle,
          CustomContext.CustomAlarm2_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM3_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm3_SHdle,
          CustomContext.CustomAlarm3_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM3_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm3_SHdle,
          CustomContext.CustomAlarm3_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM4_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm4_SHdle,
          CustomContext.CustomAlarm4_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM4_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm4_SHdle,
          CustomContext.CustomAlarm4_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM5_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm5_SHdle,
          CustomContext.CustomAlarm5_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM5_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm5_SHdle,
          CustomContext.CustomAlarm5_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM6_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm6_SHdle,
          CustomContext.CustomAlarm6_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM6_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm6_SHdle,
          CustomContext.CustomAlarm6_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM7_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm7_SHdle,
          CustomContext.CustomAlarm7_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM7_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm7_SHdle,
          CustomContext.CustomAlarm7_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM8_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm8_SHdle,
          CustomContext.CustomAlarm8_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM8_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm8_SHdle,
          CustomContext.CustomAlarm8_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM9_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm9_SHdle,
          CustomContext.CustomAlarm9_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM9_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm9_SHdle,
          CustomContext.CustomAlarm9_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM10_TIME:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm10_SHdle,
          CustomContext.CustomAlarm10_Time_CHdle, ALARM_TIME_VALUE_OFFSET, CUSTOM_STM_DATETIME_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_ALARM10_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomAlarm10_SHdle,
          CustomContext.CustomAlarm10_Status_CHdle, ALARM_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_STOPWATCH_TIME_S:
      result = aci_gatt_update_char_value(CustomContext.CustomStopwatch_SHdle,
          CustomContext.CustomStopwatch_Time_S_CHdle, STOPWATCH_TIME_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_STOPWATCH_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomStopwatch_SHdle,
          CustomContext.CustomStopwatch_Status_CHdle, STOPWATCH_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_TIMER_TIME_S:
      result = aci_gatt_update_char_value(CustomContext.CustomTimer_SHdle,
          CustomContext.CustomTimer_Time_S_CHdle, TIMER_TIME_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_TIMER_SET_TIME_S:
      result = aci_gatt_update_char_value(CustomContext.CustomTimer_SHdle,
          CustomContext.CustomTimer_Set_Time_S_CHdle, TIMER_SET_TIME_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_TIMER_STATUS:
      result = aci_gatt_update_char_value(CustomContext.CustomTimer_SHdle,
          CustomContext.CustomTimer_Status_CHdle, TIMER_STATUS_VALUE_OFFSET, CUSTOM_STM_STATUS_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_SETTINGS_LED_BLINK_INTERVAL_MS:
      result = aci_gatt_update_char_value(CustomContext.CustomSettings_SHdle,
          CustomContext.CustomSettings_Led_Blink_Interval_Ms_CHdle, SETTINGS_DURATION_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_SETTINGS_TIME_DISPLAY_DURATION_MS:
      result = aci_gatt_update_char_value(CustomContext.CustomSettings_SHdle,
          CustomContext.CustomSettings_Time_Display_Duration_Ms_CHdle, SETTINGS_DURATION_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_SETTINGS_DATE_DISPLAY_DURATION_MS:
      result = aci_gatt_update_char_value(CustomContext.CustomSettings_SHdle,
          CustomContext.CustomSettings_Date_Display_Duration_Ms_CHdle, SETTINGS_DURATION_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_SETTINGS_ALARM_RINGING_DURATION_MS:
      result = aci_gatt_update_char_value(CustomContext.CustomSettings_SHdle,
          CustomContext.CustomSettings_Alarm_Ringing_Duration_Ms_CHdle, SETTINGS_DURATION_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_SETTINGS_TIMER_RINGING_DURATION_MS:
      result = aci_gatt_update_char_value(CustomContext.CustomSettings_SHdle,
          CustomContext.CustomSettings_Timer_Ringing_Duration_Ms_CHdle, SETTINGS_DURATION_VALUE_OFFSET, CUSTOM_STM_UINT32_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_SETTINGS_DATE_FORMAT:
      result = aci_gatt_update_char_value(CustomContext.CustomSettings_SHdle,
          CustomContext.CustomSettings_Date_Format_CHdle, SETTINGS_DATE_FORMAT_VALUE_OFFSET, CUSTOM_STM_DATE_FORMAT_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_RTC_CALIBRATION_CALP:
      result = aci_gatt_update_char_value(CustomContext.CustomRtcCalibration_SHdle,
          CustomContext.CustomRtcCalibration_Calp_CHdle, RTC_CALIBRATION_CALP_VALUE_OFFSET, CUSTOM_STM_CALP_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    case CUSTOM_STM_RTC_CALIBRATION_CALM:
      result = aci_gatt_update_char_value(CustomContext.CustomRtcCalibration_SHdle,
          CustomContext.CustomRtcCalibration_Calm_CHdle, RTC_CALIBRATION_CALM_VALUE_OFFSET, CUSTOM_STM_CALM_CHAR_SIZE,
          (uint8_t*) pPayload);
      break;
    default:
      break;
  }
  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */
  /* USER CODE END Custom_STM_App_Update_Char_2 */
  return result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
