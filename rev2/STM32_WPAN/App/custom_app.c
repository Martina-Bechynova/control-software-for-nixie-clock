/* USER CODE BEGIN Header */
/**
 * @file custom_app.c
 * @brief Custom Application (Server)
 */
/*
 ******************************************************************************
 * File Name          : App/custom_app.c
 * Description        : Custom Example Application (Server)
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <limits.h>
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "lptim.h"
#include "nixie_compilation.h"
#include "stm32wbxx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  /** Notification status for the Clock Datetime characteristic. */
  uint8_t Clock_Datetime_Notification_Status;
  /** Notification status for the Stopwatch Time characteristic. */
  uint8_t Stopwatch_Time_S_Notification_Status;
  /** Notification status for the Timer Time characteristic. */
  uint8_t Timer_Time_S_Notification_Status;
  /** Notification status for the Timer Status characteristic. */
  uint8_t Timer_Status_Notification_Status;
  /** ID of the dummy timer used to keep RTC wakeup from shutting down. */
  uint8_t Dummy_Timer_Id;
  /* USER CODE END CUSTOM_APP_Context_t */
  /** Connection handle. */
  uint16_t ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/*
 * START of Section BLE_APP_CONTEXT
 */

/** Application context. */
PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/*
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */
/** Value for indicating that notifications are on. */
#define NOTIFY_ON (1)
/** Value for indicating that notifications are off. */
#define NOTIFY_OFF (0)
/** Max value the HW_TS_Start function accepts. */
#define HW_TS_START_MAX_VALUE (0xFFFF0000)

/** Application controller. */
NixieController controller;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/**
 * @brief Callback for the dummy timer.
 * @details The HW timer server turns off RTC wakeup if no timers are running on it.
 *          To prevent this from happening, the application uses a dummy timer.
 */
void Custom_Nixie_Dummy_Timer(void);

/**
 * @brief Sequencer task for GATT clock datetime notification.
 */
void Custom_Nixie_Notify_Gatt_Clock_Datetime(void);

/**
 * @brief Sequencer task for GATT stopwatch time notification.
 */
void Custom_Nixie_Notify_Gatt_Stopwatch_Time_S(void);

/**
 * @brief Sequencer task for GATT timer time notification.
 */
void Custom_Nixie_Notify_Gatt_Timer_Time_S(void);

/**
 * @brief Sequencer task for GATT timer status notification.
 */
void Custom_Nixie_Notify_Gatt_Timer_Status(void);

/**
 * @brief Sequencer task for updating the datetime member in the application controller.
 */
void Custom_Nixie_Update_Datetime(void);

/**
 * @brief Sequencer task for updating the display.
 */
void Custom_Nixie_Update_Display(void);

/**
 * @brief Sequencer task for updating LEDs.
 */
void Custom_Nixie_Update_Led(void);

/**
 * @brief Sequencer task for user alarm processing.
 */
void Custom_Nixie_Process_User_Alarm(void);

/**
 * @brief Sequencer task for timer end processing.
 */
void Custom_Nixie_Process_Timer_End(void);

/**
 * @brief Sequencer task for accelerometer activity processing.
 */
void Custom_Nixie_Process_Accelerometer_Activity(void);

/**
 * @brief Sequencer task for hourly alarm processing.
 */
void Custom_Nixie_Process_Hourly_Alarm(void);

/**
 * @brief Set application data according to new clock datetime value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Clock_Datetime(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new alarm time value.
 * @param[in,out] nc Application controller
 * @param[in] alarm_number Number of the alarm being changed
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Alarm_Time(NixieController * const nc, NixieAlarmNumber alarm_number, uint8_t * const payload);

/**
 * @brief Set application data according to new alarm status value.
 * @param[in,out] nc Application controller
 * @param[in] alarm_number Number of the alarm being changed
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Alarm_Status(NixieController * const nc, NixieAlarmNumber alarm_number, uint8_t * const payload);

/**
 * @brief Set application data according to new stopwatch status value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Stopwatch_Status(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new timer set time value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Timer_Set_Time_S(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new timer status value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Timer_Status(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new LED blink interval setting value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Settings_Led_Blink_Interval_Ms(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new time display duration setting value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Settings_Time_Display_Duration_Ms(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new date display duration setting value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Settings_Date_Display_Duration_Ms(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new alarm ringing duration setting value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Settings_Alarm_Ringing_Duration_Ms(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new timer ringing duration setting value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Settings_Timer_Ringing_Duration_Ms(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set application data according to new date format setting value.
 * @param[in,out] nc Application controller
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Settings_Date_Format(NixieController * const nc, uint8_t * const payload);

/**
 * @brief Set RTC smooth calibration CALP register according to the new value.
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Rtc_Calibration_Calp(uint8_t * const payload);

/**
 * @brief Set RTC smooth calibration CALM register according to the new value.
 * @param[in] payload Payload
 */
static void Custom_Nixie_Set_Rtc_Calibration_Calm(uint8_t * const payload);

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */
  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */
    case CUSTOM_STM_CLOCK_DATETIME_WRITE_EVT:
      Custom_Nixie_Set_Clock_Datetime(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_CLOCK_DATETIME_NOTIFY_ENABLED_EVT:
      Custom_App_Context.Clock_Datetime_Notification_Status = NOTIFY_ON;
      UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_CLOCK_DATETIME_ID, CFG_SCH_PRIO_1);
      break;
    case CUSTOM_STM_CLOCK_DATETIME_NOTIFY_DISABLED_EVT:
      Custom_App_Context.Clock_Datetime_Notification_Status = NOTIFY_OFF;
      break;
    case CUSTOM_STM_ALARM1_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM1, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM1_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM1, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM2_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM2, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM2_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM2, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM3_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM3, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM3_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM3, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM4_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM4, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM4_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM4, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM5_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM5, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM5_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM5, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM6_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM6, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM6_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM6, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM7_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM7, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM7_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM7, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM8_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM8, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM8_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM8, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM9_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM9, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM9_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM9, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM10_TIME_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Time(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM10, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_ALARM10_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Alarm_Status(&NIXIE_CONTROLLER_HANDLE, NIXIE_ALARM10, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_STOPWATCH_TIME_S_NOTIFY_ENABLED_EVT:
      Custom_App_Context.Stopwatch_Time_S_Notification_Status = NOTIFY_ON;
      UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_STOPWATCH_TIME_ID, CFG_SCH_PRIO_1);
      break;
    case CUSTOM_STM_STOPWATCH_TIME_S_NOTIFY_DISABLED_EVT:
      Custom_App_Context.Stopwatch_Time_S_Notification_Status = NOTIFY_OFF;
      break;
    case CUSTOM_STM_STOPWATCH_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Stopwatch_Status(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_TIMER_TIME_S_NOTIFY_ENABLED_EVT:
      Custom_App_Context.Timer_Time_S_Notification_Status = NOTIFY_ON;
      UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_TIME_ID, CFG_SCH_PRIO_1);
      break;
    case CUSTOM_STM_TIMER_TIME_S_NOTIFY_DISABLED_EVT:
      Custom_App_Context.Timer_Time_S_Notification_Status = NOTIFY_OFF;
      break;
    case CUSTOM_STM_TIMER_SET_TIME_S_WRITE_EVT:
      Custom_Nixie_Set_Timer_Set_Time_S(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_TIMER_STATUS_WRITE_EVT:
      Custom_Nixie_Set_Timer_Status(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_TIMER_STATUS_NOTIFY_ENABLED_EVT:
      Custom_App_Context.Timer_Status_Notification_Status = NOTIFY_ON;
      UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_STATUS_ID, CFG_SCH_PRIO_1);
      break;
    case CUSTOM_STM_TIMER_STATUS_NOTIFY_DISABLED_EVT:
      Custom_App_Context.Timer_Status_Notification_Status = NOTIFY_OFF;
      break;
    case CUSTOM_STM_SETTINGS_LED_BLINK_INTERVAL_MS_WRITE_EVT:
      Custom_Nixie_Set_Settings_Led_Blink_Interval_Ms(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_SETTINGS_TIME_DISPLAY_DURATION_MS_WRITE_EVT:
      Custom_Nixie_Set_Settings_Time_Display_Duration_Ms(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_SETTINGS_DATE_DISPLAY_DURATION_MS_WRITE_EVT:
      Custom_Nixie_Set_Settings_Date_Display_Duration_Ms(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_SETTINGS_ALARM_RINGING_DURATION_MS_WRITE_EVT:
      Custom_Nixie_Set_Settings_Alarm_Ringing_Duration_Ms(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_SETTINGS_TIMER_RINGING_DURATION_MS_WRITE_EVT:
      Custom_Nixie_Set_Settings_Timer_Ringing_Duration_Ms(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_SETTINGS_DATE_FORMAT_WRITE_EVT:
      Custom_Nixie_Set_Settings_Date_Format(&NIXIE_CONTROLLER_HANDLE, pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_RTC_CALIBRATION_CALP_WRITE_EVT:
      Custom_Nixie_Set_Rtc_Calibration_Calp(pNotification->DataTransfered.pPayload);
      break;
    case CUSTOM_STM_RTC_CALIBRATION_CALM_WRITE_EVT:
      Custom_Nixie_Set_Rtc_Calibration_Calm(pNotification->DataTransfered.pPayload);
      break;
    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */
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
    case CUSTOM_CONN_HANDLE_EVT:
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;
    case CUSTOM_DISCON_HANDLE_EVT:
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
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
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_UPDATE_DATETIME_ID, UTIL_SEQ_RFU, Custom_Nixie_Update_Datetime);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, UTIL_SEQ_RFU, Custom_Nixie_Update_Display);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, UTIL_SEQ_RFU, Custom_Nixie_Update_Led);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_PROCESS_USER_ALARM_ID, UTIL_SEQ_RFU, Custom_Nixie_Process_User_Alarm);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_PROCESS_ACCLRM_ACT_ID, UTIL_SEQ_RFU, Custom_Nixie_Process_Accelerometer_Activity);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_PROCESS_TIMER_END_ID, UTIL_SEQ_RFU, Custom_Nixie_Process_Timer_End);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_PROCESS_HOURLY_ALARM_ID, UTIL_SEQ_RFU, Custom_Nixie_Process_Hourly_Alarm);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_CLOCK_DATETIME_ID, UTIL_SEQ_RFU, Custom_Nixie_Notify_Gatt_Clock_Datetime);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_STOPWATCH_TIME_ID, UTIL_SEQ_RFU, Custom_Nixie_Notify_Gatt_Stopwatch_Time_S);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_TIME_ID, UTIL_SEQ_RFU, Custom_Nixie_Notify_Gatt_Timer_Time_S);
  UTIL_SEQ_RegTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_STATUS_ID, UTIL_SEQ_RFU, Custom_Nixie_Notify_Gatt_Timer_Status);
  
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DATETIME_ID, CFG_SCH_PRIO_0);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_0);

  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(Custom_App_Context.Dummy_Timer_Id), hw_ts_Repeated, Custom_Nixie_Dummy_Timer);
  HW_TS_Start(Custom_App_Context.Dummy_Timer_Id, HW_TS_START_MAX_VALUE);

#if (NIXIE_INIT_EEPROM == 1) && (NIXIE_EEPROM_PRESENT == 1)
  if (!nixie_init_eeprom())
  {
    Error_Handler();
  }
#endif
  if (!nixie_init_accelerometer())
  {
    Error_Handler();
  }
  nixie_init_rtc_alarms();
  nixie_enable_battery_charging();
  nixie_register_hal_callbacks();
  nixie_init_gpio();
  nixie_update_dst();
  nixie_init_controller(&NIXIE_CONTROLLER_HANDLE);
  nixie_init_gatt_server(&NIXIE_CONTROLLER_HANDLE);
  nixie_update_clock_timer(&NIXIE_CONTROLLER_HANDLE);
  nixie_update_led_timer(&NIXIE_CONTROLLER_HANDLE);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
/* USER CODE END FD */

/* ************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void Custom_Nixie_Dummy_Timer(void)
{
  return;
}

void Custom_Nixie_Notify_Gatt_Clock_Datetime(void)
{
  if (Custom_App_Context.Clock_Datetime_Notification_Status == NOTIFY_ON)
  {
    nixie_update_gatt_clock_datetime(&NIXIE_CONTROLLER_HANDLE);
  }
}

void Custom_Nixie_Notify_Gatt_Stopwatch_Time_S(void)
{
  if (Custom_App_Context.Stopwatch_Time_S_Notification_Status == NOTIFY_ON)
  {
    nixie_update_gatt_stopwatch_time_s(&NIXIE_CONTROLLER_HANDLE);
  }
}

void Custom_Nixie_Notify_Gatt_Timer_Time_S(void)
{
  if (Custom_App_Context.Timer_Time_S_Notification_Status == NOTIFY_ON)
  {
    nixie_update_gatt_timer_time_s(&NIXIE_CONTROLLER_HANDLE);
  }
}

void Custom_Nixie_Notify_Gatt_Timer_Status(void)
{
  if (Custom_App_Context.Timer_Status_Notification_Status == NOTIFY_ON)
  {
    nixie_update_gatt_timer_status(&NIXIE_CONTROLLER_HANDLE);
  }
}

void Custom_Nixie_Update_Datetime(void) 
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;

  if (HAL_RTC_GetTime(&NIXIE_RTC_HANDLE, &time, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RTC_GetDate(&NIXIE_RTC_HANDLE, &date, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  // HACK: EXTI interrupts aren't getting invoked correctly with BLE turned on, check for latched interrupt manually.
  if (HAL_GPIO_ReadPin(NIXIE_INT1_PIN_PORT, NIXIE_INT1_PIN_NUMBER) == GPIO_PIN_SET)
  {
    HAL_GPIO_EXTI_Callback(NIXIE_INT1_PIN_NUMBER);
  }

  if (!nixie_is_datetime_update_needed(&NIXIE_CONTROLLER_HANDLE, &time, &date))
  {
    return;
  }
  // HACK: Alarm interrupts aren't getting invoked correctly with BLE turned on, check flag status manually.
  if (__HAL_RTC_ALARM_GET_IT_SOURCE(&NIXIE_RTC_HANDLE, RTC_IT_ALRA) && __HAL_RTC_ALARM_GET_FLAG(&NIXIE_RTC_HANDLE, RTC_FLAG_ALRAF))
  {
    __HAL_RTC_ALARM_CLEAR_FLAG(&NIXIE_RTC_HANDLE, RTC_FLAG_ALRAF);
    NIXIE_RTC_HANDLE.AlarmAEventCallback(&NIXIE_RTC_HANDLE);
  }
  if (__HAL_RTC_ALARM_GET_IT_SOURCE(&NIXIE_RTC_HANDLE, RTC_IT_ALRB) && __HAL_RTC_ALARM_GET_FLAG(&NIXIE_RTC_HANDLE, RTC_FLAG_ALRBF))
  {
    __HAL_RTC_ALARM_CLEAR_FLAG(&NIXIE_RTC_HANDLE, RTC_FLAG_ALRBF);
    NIXIE_RTC_HANDLE.AlarmBEventCallback(&NIXIE_RTC_HANDLE);
  }
  nixie_update_rtc_year_overflow(&NIXIE_CONTROLLER_HANDLE, date.Year);
  nixie_set_gatt_clock_datetime_buffer(&NIXIE_CONTROLLER_HANDLE, &time, &date);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_CLOCK_DATETIME_ID, CFG_SCH_PRIO_1);
}

void Custom_Nixie_Process_Timer_End(void)
{
  nixie_start_ringing(&NIXIE_CONTROLLER_HANDLE, NIXIE_RINGING_SOURCE_TIMER);
}

void Custom_Nixie_Process_Accelerometer_Activity(void)
{
  nixie_process_accelerometer_activity(&NIXIE_CONTROLLER_HANDLE);
}

void Custom_Nixie_Update_Display(void)
{
  uint64_t buffer = nixie_calculate_display_data(&NIXIE_CONTROLLER_HANDLE);
  nixie_transmit_display_spi_data(buffer);
}

void Custom_Nixie_Process_User_Alarm(void)
{
  nixie_start_ringing(&NIXIE_CONTROLLER_HANDLE, NIXIE_RINGING_SOURCE_ALARM);
  nixie_reschedule_alarm(&NIXIE_CONTROLLER_HANDLE, false);
}

void Custom_Nixie_Process_Hourly_Alarm(void)
{
  nixie_process_hourly_check(&NIXIE_CONTROLLER_HANDLE);
}

void Custom_Nixie_Update_Led(void)
{
  nixie_update_leds(&NIXIE_CONTROLLER_HANDLE);
  // Display blanking is synchronised with LED blinking for a more pleasing effect.
  nixie_update_blank_pin(&NIXIE_CONTROLLER_HANDLE);
}

static void Custom_Nixie_Set_Clock_Datetime(NixieController * const nc, uint8_t * const payload)
{
  if (!nixie_is_valid_date_time(payload))
  {
    nixie_update_gatt_clock_datetime(&NIXIE_CONTROLLER_HANDLE);
    return;
  }
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  uint16_t year;

  time.Seconds = payload[NIXIE_DATETIME_SECONDS_IDX];
  time.Minutes = payload[NIXIE_DATETIME_MINUTES_IDX];
  time.Hours = payload[NIXIE_DATETIME_HOURS_IDX];
  time.SubSeconds = 0x0;

  date.Date = payload[NIXIE_DATETIME_DATE_IDX];
  date.Month = payload[NIXIE_DATETIME_MONTH_IDX];
  year = (payload[NIXIE_DATETIME_YEAR_MSB_IDX] << CHAR_BIT) | payload[NIXIE_DATETIME_YEAR_LSB_IDX];
  // RTC year can only hold year tens and units.
  date.Year = year % 100;
  date.WeekDay = nixie_calculate_day_of_week(year, date.Month, date.Date);

  if (HAL_RTC_SetDate(&NIXIE_RTC_HANDLE, &date, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RTC_SetTime(&NIXIE_RTC_HANDLE, &time, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  //nixie_set_rtc_dst_store_operation(&time, &date);
  nixie_set_datetime_members(nc, &time, &date, year);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DATETIME_ID, CFG_SCH_PRIO_0);
}

static void Custom_Nixie_Set_Alarm_Time(NixieController * const nc, NixieAlarmNumber alarm_number, uint8_t * const payload)
{
  if (!nixie_is_valid_alarm_time(payload))
  {
    nixie_update_gatt_alarm_time(nc, alarm_number);
    return;
  }
  NixieAlarmTime tmp;
  tmp.hours = payload[NIXIE_DATETIME_HOURS_IDX];
  tmp.minutes = payload[NIXIE_DATETIME_MINUTES_IDX];
  tmp.seconds = payload[NIXIE_DATETIME_SECONDS_IDX];
  nixie_set_alarm_time(nc, alarm_number, &tmp);
}

static void Custom_Nixie_Set_Alarm_Status(NixieController * const nc, NixieAlarmNumber alarm_number, uint8_t * const payload)
{
  if (!nixie_is_valid_alarm_status(payload[0]))
  {
    nixie_update_gatt_alarm_status(nc, alarm_number);
    return;
  }
  nixie_set_alarm_status(nc, alarm_number, payload[0]);
}

static void Custom_Nixie_Set_Stopwatch_Status(NixieController * const nc, uint8_t * const payload)
{
  if (nc->stopwatch_status == payload[0])
  {
    return;
  }
  switch (payload[0])
  {
    case NIXIE_STOPWATCH_OFF:
      nc->stopwatch_status = payload[0];
      nixie_stop_stopwatch_mode(nc);
      break;
    case NIXIE_STOPWATCH_ON:
      //If the display timer isn't being used by other active functions, start the timer.
      if (nc->timer_status != NIXIE_TIMER_ON)
      {
        nc->stopwatch_status = payload[0];
        nixie_start_stopwatch_mode(nc);
      }
      else
      {
        nixie_update_gatt_stopwatch_status(nc);
      }
      break;
    case NIXIE_STOPWATCH_PAUSED:
      nc->stopwatch_status = payload[0];
      nixie_pause_stopwatch_mode(nc);
      break;
    default:
      // Rewrite GATT characteristic with a valid value.
      nixie_update_gatt_stopwatch_status(nc);
      break;
  }
}

static void Custom_Nixie_Set_Timer_Set_Time_S(NixieController * const nc, uint8_t * const payload)
{
  nixie_set_timer_time_s(nc, nixie_uint8_array_to_uint32(payload));
}

static void Custom_Nixie_Set_Timer_Status(NixieController * const nc, uint8_t * const payload)
{
  if (nc->timer_status == payload[0])
  {
    return;
  }
  switch (payload[0])
  {
    case NIXIE_TIMER_OFF:
      nc->timer_status = payload[0];
      nixie_stop_timer_mode(nc);
      break;
    case NIXIE_TIMER_ON:
      //If the display timer isn't being used by other active functions, start the timer.
      if (nc->stopwatch_status != NIXIE_STOPWATCH_ON)
      {
        nc->timer_status = payload[0];
        nixie_start_timer_mode(nc);
      }
      else
      {
        nixie_update_gatt_timer_status(nc);
      }
      break;
    case NIXIE_TIMER_PAUSED:
      nc->timer_status = payload[0];
      nixie_pause_timer_mode(nc);
      break;
    default:
      // Rewrite GATT characteristic with a valid value.
      nixie_update_gatt_timer_status(nc);
      break;
  }
}

static void Custom_Nixie_Set_Settings_Led_Blink_Interval_Ms(NixieController * const nc, uint8_t * const payload)
{
  uint64_t old_value = nc->led_blink_interval_ms_ticks;
  nc->led_blink_interval_ms_ticks = nixie_calculate_led_blink_interval_ms_ticks(nixie_uint8_array_to_uint32(payload));
  if (old_value == nc->led_blink_interval_ms_ticks)
  {
    return;
  }
#if (NIXIE_EEPROM_PRESENT == 1)
  nixie_write_eeprom(NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_ADDR, payload, NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_SIZE);
#endif  
  nixie_update_led_timer(nc);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
}

static void Custom_Nixie_Set_Settings_Time_Display_Duration_Ms(NixieController * const nc, uint8_t * const payload)
{
  uint32_t old_value = nc->time_display_duration_ms;
  nc->time_display_duration_ms = nixie_uint8_array_to_uint32(payload);
  if (old_value == nc->time_display_duration_ms)
  {
    return;
  }
#if (NIXIE_EEPROM_PRESENT == 1)
  nixie_write_eeprom(NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_ADDR, payload, NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_SIZE);
#endif  
  nixie_update_clock_timer(nc);
}

static void Custom_Nixie_Set_Settings_Date_Display_Duration_Ms(NixieController * const nc, uint8_t * const payload)
{
  uint32_t old_value = nc->date_display_duration_ms;
  nc->date_display_duration_ms = nixie_uint8_array_to_uint32(payload);
  if (old_value == nc->date_display_duration_ms)
  {
    return;
  }
#if (NIXIE_EEPROM_PRESENT == 1)
  nixie_write_eeprom(NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_ADDR, payload, NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_SIZE);
#endif
  nixie_update_clock_timer(nc);
}

static void Custom_Nixie_Set_Settings_Alarm_Ringing_Duration_Ms(NixieController * const nc, uint8_t * const payload)
{
#if (NIXIE_EEPROM_PRESENT == 1)
  uint32_t old_value = nc->alarm_ringing_duration_ms_ticks;
#endif
  nc->alarm_ringing_duration_ms_ticks = nixie_calculate_ringing_duration_ms_ticks(nixie_uint8_array_to_uint32(payload));
#if (NIXIE_EEPROM_PRESENT == 1)
  if (old_value != nc->alarm_ringing_duration_ms_ticks)
  {
    nixie_write_eeprom(NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_ADDR, payload, NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_SIZE);
  }
#endif
}

static void Custom_Nixie_Set_Settings_Timer_Ringing_Duration_Ms(NixieController * const nc, uint8_t * const payload)
{
#if (NIXIE_EEPROM_PRESENT == 1)
  uint32_t old_value = nc->timer_ringing_duration_ms_ticks;
#endif
  nc->timer_ringing_duration_ms_ticks = nixie_calculate_ringing_duration_ms_ticks(nixie_uint8_array_to_uint32(payload));
#if (NIXIE_EEPROM_PRESENT == 1)
  if (old_value != nc->timer_ringing_duration_ms_ticks)
  {
    nixie_write_eeprom(NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_ADDR, payload, NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_SIZE);
  }
#endif
}

static void Custom_Nixie_Set_Settings_Date_Format(NixieController * const nc, uint8_t * const payload)
{
  if (!nixie_is_valid_date_format(payload[0]))
  {
    nixie_update_gatt_settings_date_format(nc);
    return;
  }
#if (NIXIE_EEPROM_PRESENT == 1)
  if (nc->date_format != payload[0])
  {
    nixie_write_eeprom(NIXIE_EEPROM_DATE_FORMAT_ADDR, payload, NIXIE_EEPROM_DATE_FORMAT_SIZE);
  }
#endif
  nc->date_format = payload[0];
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
}

static void Custom_Nixie_Set_Rtc_Calibration_Calp(uint8_t * const payload)
{
  if (!nixie_is_valid_rtc_calibration_calp(payload[0]))
  {
    nixie_update_gatt_rtc_calibration_calp();
    return;
  }
  uint32_t calp = payload[0]? RTC_SMOOTHCALIB_PLUSPULSES_SET: RTC_SMOOTHCALIB_PLUSPULSES_RESET;
  if (HAL_RTCEx_SetSmoothCalib(&NIXIE_RTC_HANDLE, RTC_SMOOTHCALIB_PERIOD_32SEC, calp, LL_RTC_CAL_GetMinus(NIXIE_RTC_HANDLE.Instance)))
  {
    Error_Handler();
  }
}

static void Custom_Nixie_Set_Rtc_Calibration_Calm(uint8_t * const payload)
{
  uint16_t calm = nixie_uint8_array_to_uint16(payload);
  if (!IS_RTC_SMOOTH_CALIB_MINUS(calm))
  {
    nixie_update_gatt_rtc_calibration_calm();
    return;
  }
  uint32_t calp = LL_RTC_CAL_IsPulseInserted(NIXIE_RTC_HANDLE.Instance)? RTC_SMOOTHCALIB_PLUSPULSES_SET: RTC_SMOOTHCALIB_PLUSPULSES_RESET;
  if (HAL_RTCEx_SetSmoothCalib(&NIXIE_RTC_HANDLE, RTC_SMOOTHCALIB_PERIOD_32SEC, calp, calm))
  {
    Error_Handler();
  }
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
