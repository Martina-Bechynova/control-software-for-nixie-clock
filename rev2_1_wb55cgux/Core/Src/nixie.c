/**
 * @file nixie.c
 * @brief Source code file for nixie_ functions.
 */

#include <limits.h>
#include "rtc.h"
#include "lptim.h"
#include "tim.h"
#include "i2c.h"
#include "spi.h"
#include "custom_app.h"
#include "stm32_seq.h"
#include "nixie.h"
#include "nixie_compilation.h"

// Macros for HAL functions
/** Resistor used for battery charging. */
#define NIXIE_BATTERY_CHARGING_RESISTOR (PWR_BATTERY_CHARGING_RESISTOR_1_5)
/** RTC store operation value used to indicate the RTC is currently using standard time. */
#define NIXIE_RTC_DST_STORE_STANDARD_TIME (RTC_STOREOPERATION_RESET)
/** RTC store operation value used to indicate the RTC is currently using summer time. */
#define NIXIE_RTC_DST_STORE_SUMMER_TIME (RTC_STOREOPERATION_SET)
/** The number of data transfers it takes to send all display data. */
#define NIXIE_DISPLAY_SPI_ARRAY_SIZE (4)
/** The bit size of a single display SPI transfer. */
#define NIXIE_DISPLAY_SPI_DATA_BITSIZE (16)
/** TIM channel used for PWM generation. */
#define NIXIE_PWM_TIM_CHANNEL (TIM_CHANNEL_4)
/** Size of the memory address section of the I2C transfer for the accelerometer. */
#define NIXIE_ACCLRM_MEMADD_SIZE (I2C_MEMADD_SIZE_8BIT)
/** Timeout used on blocking accelerometer operations. */
#define NIXIE_ACCLRM_TIMEOUT (1000)
/** Size of the memory address section of the I2C transfer for the EEPROM. */
#define NIXIE_EEPROM_MEMADD_SIZE (I2C_MEMADD_SIZE_8BIT)
/** Timeout used on blocking EEPROM operations. */
#define NIXIE_EEPROM_TIMEOUT (1000)

/** Maximum write cycle time of the EEPROM */
#define NIXIE_EEPROM_WRITE_CYCLE_MS (5U)
/** Byte size of the EEPROM page. */
#define NIXIE_EEPROM_PAGE_SIZE (16U)

/** Default LED blink interval value. */
#define NIXIE_LED_BLINK_INTERVAL_DEFAULT_VALUE (500U)
/** Default time display duration value. */
#define NIXIE_TIME_DISPLAY_DURATION_DEFAULT_VALUE (5000U)
/** Default date display duration value. */
#define NIXIE_DATE_DISPLAY_DURATION_DEFAULT_VALUE (2000U)
/** Default alarm ringing duration value. */
#define NIXIE_ALARM_RINGING_DURATION_DEFAULT_VALUE (15000U)
/** Default timer ringing duration value. */
#define NIXIE_TIMER_RINGING_DURATION_DEFAULT_VALUE (10000U)
/** Default date format value. */
#define NIXIE_DATE_FORMAT_DEFAULT_VALUE (NIXIE_DATE_FORMAT_EU)
/** Default year hundreds value. */
#define NIXIE_YEAR_HUNDREDS_DEFAULT_VALUE (20U)

/** Shift register value that can be used to make the display empty (on devices with the HV5623 converter). */
#define NIXIE_EMPTY_DISPLAY_HV5623 (0x1ULL)
/** Shift register value that can be used to make the display empty (on devices with the HV5523 converter). */
#define NIXIE_EMPTY_DISPLAY_HV5523 (0x8000ULL)

/** Display blink interval value. Used during ringing if LED blink interval is set to 0. */
#define NIXIE_DISPLAY_BLINK_INTERVAL_MS_VALUE (500U)

/** PWM frequency used for buzzing. */
#define NIXIE_PWM_FREQUENCY_HZ (2300)
/** Max TIM ARR value of the timer used for PWM generation. */
#define NIXIE_PWM_TIM_ARR_MAX (0xFFFFU)
/** PWM duty cycle percentage (i.e. value should be 0 - 100). */
#define NIXIE_PWM_DUTY_CYCLE (50)


/** Number of days in March. */
#define NIXIE_DAYS_IN_MARCH (31)
/** Number of days in October. */
#define NIXIE_DAYS_IN_OCTOBER (31)

/** Number of detoxification rounds (a round consists of cycling through all digits). */
#define NIXIE_DETOX_ROUNDS_NUM (6)
/** Time period of how long a digit gets lit up in a round. */
#define NIXIE_DETOX_DIGIT_DURATION_MS (500U)

/** Size of the byte array used during EEPROM initialization. It should be large enough to fit the biggest data type saved in the EEPROM. */
#define NIXIE_EEPROM_INIT_ARRAY_SIZE (CUSTOM_STM_UINT32_CHAR_SIZE)

/** LPTIM update type. */
typedef enum
{
  /** Update LED blinking LPTIM flag. */
  NIXIE_UPDATE_LED_BLINK_LPTIM,
  /** Update alarm ringing LPTIM flag. */
  NIXIE_UPDATE_ALARM_RINGING_LPTIM,
  /** Update timer ringing LPTIM flag. */
  NIXIE_UPDATE_TIMER_RINGING_LPTIM,
  /** Update display blinking LPTIM flag. */
  NIXIE_UPDATE_DISPLAY_BLINK_LPTIM
} NixieLptimUpdateType;

/** OPT offset array indexes for time displaying. */
typedef enum
{
  /** Hours tens OPT offset array index. */
  NIXIE_HOURS_TENS_OPT = 0,
  /** Hours units OPT offset array index. */
  NIXIE_HOURS_UNITS_OPT,
  /** Minutes tens OPT offset array index. */
  NIXIE_MINUTES_TENS_OPT,
  /** Minutes units OPT offset array index. */
  NIXIE_MINUTES_UNITS_OPT,
  /** Seconds tens OPT offset array index. */
  NIXIE_SECONDS_TENS_OPT,
  /** Seconds units OPT offset array index. */
  NIXIE_SECONDS_UNITS_OPT,
} NixieDisplayTimeOpt;

/** OPT offset array indexes for date displaying. */
typedef enum
{
  /** Date tens OPT offset array index (EU format). */
  NIXIE_EU_DATE_TENS_OPT = 0,
  /** Date units OPT offset array index (EU format). */
  NIXIE_EU_DATE_UNITS_OPT,
  /** Month tens OPT offset array index (EU format). */
  NIXIE_EU_MONTH_TENS_OPT,
  /** Month units OPT offset array index (EU format). */
  NIXIE_EU_MONTH_UNITS_OPT,
  /** Year tens OPT offset array index (EU format). */
  NIXIE_EU_YEAR_TENS_OPT,
  /** Year units OPT offset array index (EU format). */
  NIXIE_EU_YEAR_UNITS_OPT,
  /** Month tens OPT offset array index (US format). */
  NIXIE_US_MONTH_TENS_OPT = 0,
  /** Month units OPT offset array index (US format). */
  NIXIE_US_MONTH_UNITS_OPT,
  /** Date tens OPT offset array index (US format). */
  NIXIE_US_DATE_TENS_OPT,
  /** Date units OPT offset array index (US format). */
  NIXIE_US_DATE_UNITS_OPT,
  /** Year tens OPT offset array index (US format). */
  NIXIE_US_YEAR_TENS_OPT,
  /** Year units OPT offset array index (US format). */
  NIXIE_US_YEAR_UNITS_OPT,
} NixieDisplayDateOpt;

/** Return status of the nixie_set_digit_array_clock function */
typedef enum
{
  /** Flag indicating the display should be empty. */
  NIXIE_DIGIT_ARRAY_EMPTY,
  /** Flag indicating the display shouldn't be empty. */
  NIXIE_DIGIT_ARRAY_NOT_EMPTY,
} NixieSetDigitArrayStatus;

/** Expected DST state calculation type */
typedef enum
{
  /** The calculation is done to check for DST updates */
  NIXIE_DST_CALCULATION_UPDATE,
  /** The calculation is done to set DST bit, used when setting up RTC with new time. */
  NIXIE_DST_CALCULATION_SET
} NixieDstCalculationType;

/**
 * @brief Update event callback for the display timer in detoxification mode.
 * @param[in,out] tim HAL TIM handle
 */
void nixie_display_timer_detoxification_cb(TIM_HandleTypeDef *tim);

/**
 * @brief Update event callback for the display timer in timer mode.
 * @param[in,out] tim HAL TIM handle
 */
void nixie_display_timer_timer_cb(TIM_HandleTypeDef *tim);

/**
 * @brief Update event callback for the display timer in stopwatch mode.
 * @param[in,out] tim HAL TIM handle
 */
void nixie_display_timer_stopwatch_cb(TIM_HandleTypeDef *tim);

/**
 * @brief Update event callback for the display timer in clock mode.
 * @param[in,out] tim HAL TIM handle
 */
void nixie_display_timer_clock_cb(TIM_HandleTypeDef *tim);

/**
 * @brief Autoreload reload event callback for the timer ringing timer when the tick value > ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_ringing_timer_timer_cb_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief Autoreload reload event callback for the timer ringing timer when the tick value <= ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_ringing_timer_timer_cb_no_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief Autoreload reload event callback for the alarm ringing timer when the tick value > ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_ringing_timer_alarm_cb_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief Autoreload reload event callback for the alarm ringing timer when the tick value <= ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_ringing_timer_alarm_cb_no_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief Autoreload reload event callback for the display blanking timer when the tick value > ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_blank_timer_cb_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief Autoreload reload event callback for the display blanking timer when the tick value <= ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_blank_timer_cb_no_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief Autoreload reload event callback for the LED blinking timer when the tick value > ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_led_timer_cb_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief Autoreload reload event callback for the LED blinking timer when the tick value <= ARR max value.
 * @param[in,out] lptim HAL LPTIM handle
 */
void nixie_led_timer_cb_no_overflow(LPTIM_HandleTypeDef *lptim);

/**
 * @brief RTC alarm event callback for the user alarm.
 * @param[in,out] rtc HAL RTC handle
 */
void nixie_rtc_user_alarm_cb(RTC_HandleTypeDef *rtc);

/**
 * @brief RTC alarm event callback for the hourly alarm.
 * @param[in,out] rtc HAL RTC handle
 */
void nixie_rtc_hourly_alarm_cb(RTC_HandleTypeDef *rtc);

/**
 * @brief RTC wakeup event callback.
 * @param[in,out] rtc HAL RTC handle
 */
void nixie_rtc_wakeup_cb(RTC_HandleTypeDef *rtc);

/**
 * @brief Transmission complete event callback for the display SPI.
 * @param[in,out] spi HAL SPI handle
 */
void nixie_display_spi_cb(SPI_HandleTypeDef *spi);

/**
 * The offset of the first of the ten digits corresponding to a specific tube (OPT) (on devices with the HV5623 converter).
 * The offset is counted from the 1st bit in the "lower" converter.
 * arr[0] = offset of OPT1, arr[1] = offset of OPT2,...
 */
static uint8_t const nixie_opt_offset_hv5623[NIXIE_OPT_NUM] = {2U, 12U, 22U, 34U, 44U, 54U};

/**
 * Digit offset from the OPT (tube number) offset value (on devices with the HV5623 converter).
 * The offset is counted from the 1st bit in the "lower" converter.
 * arr[0] = offset of digit 0, arr[1] = offset of digit 1,...
 */
static uint8_t const nixie_digit_offset_hv5623[NIXIE_DIGITS_NUM] = {9U, 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};

/**
 * The offset of the first of the ten digits corresponding to a specific tube (OPT) (on devices with the HV5523 converter).
 * The offset is counted from the 1st bit in the "lower" converter.
 * arr[0] = offset of OPT1, arr[1] = offset of OPT2,...
 */
static uint8_t const nixie_opt_offset_hv5523[NIXIE_OPT_NUM] = {20U, 10U, 0U, 52U, 42U, 32U};

/** 
 * Digit offset from the OPT (tube number) offset value (on devices with the HV5523 converter). 
 * The offset is counted from the 1st bit in the "lower" converter.
 * arr[0] = offset of digit 0, arr[1] = offset of digit 1,...
 */
static uint8_t const nixie_digit_offset_hv5523[NIXIE_DIGITS_NUM] = {0U, 9U, 8U, 7U, 6U, 5U, 4U, 3U, 2U, 1U};

/**
 * @brief Remove an alarm from the active alarms array.
 * @param[in,out] nc Application controller
 * @param[in] time Alarm time
 */
static void nixie_remove_alarm(NixieController * const nc, NixieAlarmTime const * const time);

/**
 * @brief Add an alarm to the active alarms array.
 * @param[in,out] nc Application controller
 * @param[in] time Alarm time
 */
static void nixie_add_alarm(NixieController * const nc, NixieAlarmTime const * const time);

/**
 * @brief Compare two alarm values.
 * @param[in] a First value
 * @param[in] b Second value
 * @return 1 if a > b, 0 if a == b, -1 if a < b
 */
static int nixie_compare_nixiealarmtime(NixieAlarmTime const * const a, NixieAlarmTime const * const b);

/**
 * @brief Calculate LPTIM ticks needed to reach the desired display blinking interval in ms.
 * @param[in] display_blink_interval_ms Display blink interval in milliseconds
 * @return Timer ticks
 */
static uint64_t nixie_calculate_display_blink_interval_ms_ticks(uint32_t display_blink_interval_ms);

/**
 * @brief Calculate LPTIM ticks needed to reach a specific time period in ms.
 * @param[in] lptim HAL LPTIM handle
 * @param[in] time_period_ms Time period in milliseconds
 * @return Timer ticks
 */
static uint64_t nixie_calculate_lptim_ticks_ms(const LPTIM_HandleTypeDef * const lptim, uint32_t time_period_ms);

/**
 * @brief Calculacte the time period in ms the LPTIM ticks are equal to.
 * @param[in] lptim HAL LPTIM handle for which the ticks were calculated
 * @param[in] period_ticks_ms LPTIM period ticks in milliseconds
 * @return Time period in ms
 */
static uint32_t nixie_calculate_period_lptim_ms(const LPTIM_HandleTypeDef * const lptim, uint64_t period_ticks_ms);

/**
 * @brief Calculate TIM ticks needed to reach a specific time period in ms.
 * @param[in] tim HAL TIM handle
 * @param[in] time_period_ms Time period in ms
 * @return Timer ticks
 */
static uint64_t nixie_calculate_tim_ticks_ms(const TIM_HandleTypeDef * const tim, uint32_t time_period_ms);

/**
 * @brief Update the desired LPTIM timer.
 * @param[in,out] nc Application controller
 * @param[in] update_type Flag indicating which LPTIM timer to update.
 */
static void nixie_update_lptim_timer(NixieController * const nc, NixieLptimUpdateType update_type);

/**
 * @brief Calculate the expected RTC store operation register value.
 * @param[in] time RTC time
 * @param[in] date RTC date
 * @param[in] type DST calculating type
 * @return A value of @ref RTC_StoreOperation_Definitions
 */
static uint32_t nixie_calculate_dst_state(RTC_TimeTypeDef const * const time, RTC_DateTypeDef const * const date, NixieDstCalculationType type);

/**
 * @brief Start the PWM timer.
 */
static void nixie_start_pwm_timer(void);

/**
 * @brief Stop the PWM timer.
 */
static void nixie_stop_pwm_timer(void);

/**
 * @brief Fill the supplied array with clock data, depending on the clock state.
 * @param[in] nc Application controller
 * @param[out] array Array used to store which digits to light on the display
 * @return Flag indicating whether the display contains any data or is completely empty
 */
static NixieSetDigitArrayStatus nixie_set_digit_array_clock(NixieController const * const nc, uint8_t array[const NIXIE_OPT_NUM]);

/**
 * @brief Convert the seconds into display data and set the digit array accordingly.
 * @param[out] array Array used to store which digits to light on the display
 * @param[in] seconds Display data in seconds. Will be converted to HH:MM:SS format.
 */
static void nixie_set_digit_array_seconds(uint8_t array[const NIXIE_OPT_NUM], uint32_t seconds);

/**
 * @brief Calculates display SPI data (for devices using the HV5623 converter).
 * @param[in] array Information on which digit to light up on each tube.
 * @return Display SPI data (in MSB order)
 */
static uint64_t nixie_calculate_display_data_hv5623(uint8_t const array[const NIXIE_OPT_NUM]);

/**
 * @brief Calculates display SPI data (for devices using the HV5623 converter).
 * @param[in] array Information on which digit to light up on each tube.
 * @return Display SPI data (in MSB order)
 */
static uint64_t nixie_calculate_display_data_hv5523(uint8_t const array[const NIXIE_OPT_NUM]);

/**
 * @brief Detoxify the nixie tube display.
 * @param[in,out] nc Application controller
 */
static void nixie_detoxify(NixieController * const nc);

// ----------------------------------------

void nixie_uint32_to_uint8_array(uint8_t array[const NIXIE_UINT32_SIZE], uint32_t value)
{
  for (unsigned int i = 0; i < NIXIE_UINT32_SIZE; ++i)
  {
#if (NIXIE_GATT_SERVER_ENDIANNESS == NIXIE_GATT_SERVER_BIG_ENDIAN)
    array[i] = (value >> ((NIXIE_UINT32_SIZE - 1 - i) * CHAR_BIT)) & 0xFF;
#else
    array[i] = (value >> (i * CHAR_BIT)) & 0xFF;
#endif
  }
}

uint32_t nixie_uint8_array_to_uint32(uint8_t const array[const NIXIE_UINT32_SIZE])
{
  uint32_t rv = 0;
  for (unsigned int i = 0; i < NIXIE_UINT32_SIZE; ++i)
  {
#if (NIXIE_GATT_SERVER_ENDIANNESS == NIXIE_GATT_SERVER_BIG_ENDIAN)
    rv |= array[i] << (CHAR_BIT * (NIXIE_UINT32_SIZE - 1 - i));
#else
    rv |= array[i] << (CHAR_BIT * i);
#endif
  }
  return rv;
}

void nixie_uint16_to_uint8_array(uint8_t array[const NIXIE_UINT16_SIZE], uint16_t value)
{
  for (unsigned int i = 0; i < NIXIE_UINT16_SIZE; ++i)
  {
#if (NIXIE_GATT_SERVER_ENDIANNESS == NIXIE_GATT_SERVER_BIG_ENDIAN)
    array[i] = (value >> ((NIXIE_UINT16_SIZE - 1 - i) * CHAR_BIT)) & 0xFF;
#else
    array[i] = (value >> (i * CHAR_BIT)) & 0xFF;
#endif
  }
}

uint16_t nixie_uint8_array_to_uint16(uint8_t const array[const NIXIE_UINT16_SIZE])
{
  uint16_t rv = 0;
  for (unsigned int i = 0; i < NIXIE_UINT16_SIZE; ++i)
  {
#if (NIXIE_GATT_SERVER_ENDIANNESS == NIXIE_GATT_SERVER_BIG_ENDIAN)
    rv |= array[i] << (CHAR_BIT * (NIXIE_UINT16_SIZE - 1 - i));
#else
    rv |= array[i] << (CHAR_BIT * i);
#endif
  }
  return rv;
}

void nixie_init_gpio(void)
{
  HAL_GPIO_WritePin(NIXIE_LED1_PIN_PORT, NIXIE_LED1_PIN_NUMBER, NIXIE_LED_PIN_OFF);
  HAL_GPIO_WritePin(NIXIE_LED2_PIN_PORT, NIXIE_LED2_PIN_NUMBER, NIXIE_LED_PIN_OFF);
  HAL_GPIO_WritePin(NIXIE_LED3_PIN_PORT, NIXIE_LED3_PIN_NUMBER, NIXIE_LED_PIN_OFF);
  HAL_GPIO_WritePin(NIXIE_LED4_PIN_PORT, NIXIE_LED4_PIN_NUMBER, NIXIE_LED_PIN_OFF);
  HAL_GPIO_WritePin(NIXIE_LATCH_PIN_PORT, NIXIE_LATCH_PIN_NUMBER, NIXIE_LATCH_PIN_NOT_LATCHED);
  HAL_GPIO_WritePin(NIXIE_BLANK_PIN_PORT, NIXIE_BLANK_PIN_NUMBER, NIXIE_BLANK_PIN_NOT_BLANK);
}

void nixie_enable_battery_charging(void)
{
  HAL_PWREx_EnableBatteryCharging(NIXIE_BATTERY_CHARGING_RESISTOR);
}

void nixie_display_spi_cb(SPI_HandleTypeDef *spi)
{
  UNUSED(spi);
  HAL_GPIO_WritePin(NIXIE_LATCH_PIN_PORT, NIXIE_LATCH_PIN_NUMBER, NIXIE_LATCH_PIN_NOT_LATCHED);
}

void nixie_display_timer_clock_cb(TIM_HandleTypeDef *tim)
{
  if (NIXIE_CONTROLLER_HANDLE.clock_state == NIXIE_CLOCK_STATE_TIME)
  {
    NIXIE_CONTROLLER_HANDLE.clock_state = NIXIE_CLOCK_STATE_DATE;
    __HAL_TIM_SET_AUTORELOAD(tim, NIXIE_CONTROLLER_HANDLE.date_display_duration_ms - 1);
  }
  // If timer is running the clock state can't be NIXIE_CLOCK_NONE.
  else
  {
    NIXIE_CONTROLLER_HANDLE.clock_state = NIXIE_CLOCK_STATE_TIME;
    __HAL_TIM_SET_AUTORELOAD(tim, NIXIE_CONTROLLER_HANDLE.time_display_duration_ms - 1);
  }
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_0);
}

void nixie_rtc_user_alarm_cb(RTC_HandleTypeDef *rtc)
{
  UNUSED(rtc);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_PROCESS_USER_ALARM_ID, CFG_SCH_PRIO_0);
}

void nixie_rtc_hourly_alarm_cb(RTC_HandleTypeDef *rtc)
{
  UNUSED(rtc);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_PROCESS_HOURLY_ALARM_ID, CFG_SCH_PRIO_0);
}

void nixie_rtc_wakeup_cb(RTC_HandleTypeDef *rtc)
{
  UNUSED(rtc);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DATETIME_ID, CFG_SCH_PRIO_0);
}

void nixie_display_timer_stopwatch_cb(TIM_HandleTypeDef *tim)
{
  UNUSED(tim);
  ++NIXIE_CONTROLLER_HANDLE.stopwatch_time_s;
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_STOPWATCH_TIME_ID, CFG_SCH_PRIO_1);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
}

void nixie_display_timer_timer_cb(TIM_HandleTypeDef *tim)
{
  --NIXIE_CONTROLLER_HANDLE.timer_time_s;
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_TIME_ID, CFG_SCH_PRIO_1);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  if (!NIXIE_CONTROLLER_HANDLE.timer_time_s)
  {
    if (HAL_TIM_Base_Stop_IT(tim) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.timer_status = NIXIE_TIMER_OFF;
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_STATUS_ID, CFG_SCH_PRIO_1);
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_PROCESS_TIMER_END_ID, CFG_SCH_PRIO_0);
  }
}

void nixie_display_timer_detoxification_cb(TIM_HandleTypeDef *tim)
{
  if (NIXIE_CONTROLLER_HANDLE.detoxification_idx)
  {
    --NIXIE_CONTROLLER_HANDLE.detoxification_idx;
  }
  else
  {
    if (HAL_TIM_Base_Stop_IT(tim) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.display_state = NIXIE_DISPLAY_STATE_CLOCK;
    nixie_update_clock_timer(&NIXIE_CONTROLLER_HANDLE);
    return;
  }
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
}

void nixie_led_timer_cb_no_overflow(LPTIM_HandleTypeDef *lptim)
{
  UNUSED(lptim);
  if (NIXIE_CONTROLLER_HANDLE.led_state == NIXIE_LED_STATE_ON)
  {
    NIXIE_CONTROLLER_HANDLE.led_state = NIXIE_LED_STATE_OFF;
    NIXIE_CONTROLLER_HANDLE.blank_state = NIXIE_BLANK_STATE_ON;
  }
  else
  {
    NIXIE_CONTROLLER_HANDLE.led_state = NIXIE_LED_STATE_ON;
    NIXIE_CONTROLLER_HANDLE.blank_state = NIXIE_BLANK_STATE_OFF;
  }
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_0);
}

void nixie_led_timer_cb_overflow(LPTIM_HandleTypeDef *lptim)
{
  if (NIXIE_CONTROLLER_HANDLE.led_blink_ticks_left > LPTIM_ARR_ARR)
  {
    // Current ARR is 0xFFFF, so no need to update LPTIM settings.
    NIXIE_CONTROLLER_HANDLE.led_blink_ticks_left -= LPTIM_ARR_ARR;
  }
  else if (NIXIE_CONTROLLER_HANDLE.led_blink_ticks_left)
  {
    if (HAL_LPTIM_Counter_Stop_IT(lptim) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_LPTIM_Counter_Start_IT(lptim, NIXIE_CONTROLLER_HANDLE.led_blink_ticks_left) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.led_blink_ticks_left = 0;
  }
  else
  {
    if (HAL_LPTIM_Counter_Stop_IT(lptim) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.led_blink_ticks_left = NIXIE_CONTROLLER_HANDLE.led_blink_interval_ms_ticks - LPTIM_ARR_ARR;
    if (HAL_LPTIM_Counter_Start_IT(lptim, LPTIM_ARR_ARR) != HAL_OK)
    {
      Error_Handler();
    }
    // After the total ticks not fitting into the timer is resolved, the interrupt does the same as the no OF version.
    nixie_led_timer_cb_no_overflow(lptim);
  }
}

void nixie_blank_timer_cb_no_overflow(LPTIM_HandleTypeDef *lptim)
{
  UNUSED(lptim);
  if (NIXIE_CONTROLLER_HANDLE.blank_state == NIXIE_BLANK_STATE_ON)
  {
    NIXIE_CONTROLLER_HANDLE.blank_state = NIXIE_BLANK_STATE_OFF;
  }
  else
  {
    NIXIE_CONTROLLER_HANDLE.blank_state = NIXIE_BLANK_STATE_ON;
  }
  nixie_update_blank_pin(&NIXIE_CONTROLLER_HANDLE);
}

void nixie_blank_timer_cb_overflow(LPTIM_HandleTypeDef *lptim)
{
  if (NIXIE_CONTROLLER_HANDLE.display_blink_ticks_left > LPTIM_ARR_ARR)
  {
    // Current ARR is 0xFFFF, so no need to update LPTIM settings.
    NIXIE_CONTROLLER_HANDLE.display_blink_ticks_left -= LPTIM_ARR_ARR;
  }
  else if (NIXIE_CONTROLLER_HANDLE.display_blink_ticks_left)
  {
    if (HAL_LPTIM_Counter_Stop_IT(lptim) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_LPTIM_Counter_Start_IT(lptim, NIXIE_CONTROLLER_HANDLE.display_blink_ticks_left) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.display_blink_ticks_left = 0;
  }
  else
  {
    if (HAL_LPTIM_Counter_Stop_IT(lptim) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.display_blink_ticks_left = NIXIE_CONTROLLER_HANDLE.display_blink_interval_ms_ticks - LPTIM_ARR_ARR;
    if (HAL_LPTIM_Counter_Start_IT(lptim, LPTIM_ARR_ARR) != HAL_OK)
    {
      Error_Handler();
    }
    // After the total ticks not fitting into the timer is resolved, the interrupt does the same as the no OF version.
    nixie_blank_timer_cb_no_overflow(lptim);
  }
}

void nixie_ringing_timer_alarm_cb_no_overflow(LPTIM_HandleTypeDef *lptim)
{
  UNUSED(lptim);
  nixie_stop_ringing(&NIXIE_CONTROLLER_HANDLE);
}

void nixie_ringing_timer_alarm_cb_overflow(LPTIM_HandleTypeDef *lptim)
{
  if (NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left > LPTIM_ARR_ARR)
  {
    // Current ARR is 0xFFFF, so no need to update LPTIM settings.
    NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left -= LPTIM_ARR_ARR;
  }
  else if (NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left)
  {
    if (HAL_LPTIM_Counter_Stop_IT(lptim) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_LPTIM_Counter_Start_IT(lptim, NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left = 0;
  }
  else
  {
    // Ringing doesn't repeat, no need to restart timer.
    nixie_ringing_timer_alarm_cb_no_overflow(lptim);
  }
}

void nixie_ringing_timer_timer_cb_no_overflow(LPTIM_HandleTypeDef *lptim)
{
  UNUSED(lptim);
  nixie_stop_ringing(&NIXIE_CONTROLLER_HANDLE);
}

void nixie_ringing_timer_timer_cb_overflow(LPTIM_HandleTypeDef *lptim)
{
  if (NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left > LPTIM_ARR_ARR)
  {
    // Current ARR is 0xFFFF, so no need to update LPTIM settings.
    NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left -= LPTIM_ARR_ARR;
  }
  else if (NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left)
  {
    if (HAL_LPTIM_Counter_Stop_IT(lptim) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_LPTIM_Counter_Start_IT(lptim, NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left) != HAL_OK)
    {
      Error_Handler();
    }
    NIXIE_CONTROLLER_HANDLE.ringing_duration_ticks_left = 0;
  }
  else
  {
    // Ringing doesn't repeat, no need to restart timer.
    nixie_ringing_timer_timer_cb_no_overflow(lptim);
  }
}

uint64_t nixie_calculate_led_blink_interval_ms_ticks(uint32_t led_blink_interval_ms)
{
  return nixie_calculate_lptim_ticks_ms(&NIXIE_LED_TIM_HANDLE, led_blink_interval_ms);
}

uint64_t nixie_calculate_ringing_duration_ms_ticks(uint32_t ringing_duration_ms)
{
  return nixie_calculate_lptim_ticks_ms(&NIXIE_RINGING_TIM_HANDLE, ringing_duration_ms);
}

void nixie_register_hal_callbacks(void)
{
  if (HAL_SPI_RegisterCallback(&NIXIE_DISPLAY_SPI_HANDLE, HAL_SPI_TX_COMPLETE_CB_ID, nixie_display_spi_cb) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RTC_RegisterCallback(&NIXIE_RTC_HANDLE, HAL_RTC_WAKEUPTIMER_EVENT_CB_ID, nixie_rtc_wakeup_cb) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t nixie_calculate_day_of_week(unsigned int year, unsigned int month, unsigned int date)
{
  // This function uses Zeller's congruence for the calculation. It has been slightly modified using modulo arithmetics (-2J to +5J) to avoid negative values.
  if (month < 3) {
    month += 12;
    --year;
  }
  unsigned int year_hundreds = year / 100;
  year %= 100;
  unsigned int rv = (date + (month + 1) * 26 / 10 + year + year / 4 + year_hundreds / 4 + 5 * year_hundreds) % 7;
  // Zeller's algorithm returns 0 = Sat, 1 = Sun... but the RTC uses ISO values. The conversion formula is (rv + 5) mod 7 + 1.
  return (rv + 5) % 7 + 1;
}

void nixie_init_controller(NixieController * const nc)
{
  if (!nc)
  {
    return;
  }

  for (unsigned i = 0; i < CUSTOM_STM_DATETIME_CHAR_SIZE; ++i)
  {
    nc->gatt_clock_datetime_buffer[i] = 0;
  }
  nc->year_hundreds = NIXIE_YEAR_HUNDREDS_DEFAULT_VALUE;
  nc->prev_rtc_year = 0;
  nc->stopwatch_time_s = 0;
  nc->stopwatch_tim_cnt = 0;
  nc->stopwatch_status = NIXIE_STOPWATCH_OFF;
  nc->timer_tim_cnt = 0;
  nc->timer_time_s = 0;
  nc->timer_status = NIXIE_TIMER_OFF;

  for (int i = 0; i < NIXIE_ALARMS_MAX; ++i)
  {
    nc->alarms_info[i].time.seconds = 0;
    nc->alarms_info[i].time.minutes = 0;
    nc->alarms_info[i].time.hours = 0;
    nc->alarms_info[i].status = NIXIE_ALARM_OFF;

    nc->active_alarms[i].time.seconds = 0;
    nc->active_alarms[i].time.minutes = 0;
    nc->active_alarms[i].time.hours = 0;
    nc->active_alarms[i].cnt = 0;
  }
  nc->active_alarms_scheduled_idx = 0;
  nc->active_alarms_size = 0;

  nc->led_blink_interval_ms_ticks = nixie_calculate_led_blink_interval_ms_ticks(NIXIE_LED_BLINK_INTERVAL_DEFAULT_VALUE);
  nc->time_display_duration_ms = NIXIE_TIME_DISPLAY_DURATION_DEFAULT_VALUE;
  nc->date_display_duration_ms = NIXIE_DATE_DISPLAY_DURATION_DEFAULT_VALUE;
  nc->alarm_ringing_duration_ms_ticks = nixie_calculate_ringing_duration_ms_ticks(NIXIE_ALARM_RINGING_DURATION_DEFAULT_VALUE);
  nc->timer_ringing_duration_ms_ticks = nixie_calculate_ringing_duration_ms_ticks(NIXIE_TIMER_RINGING_DURATION_DEFAULT_VALUE);
  nc->date_format = NIXIE_DATE_FORMAT_DEFAULT_VALUE;

  nc->led_blink_ticks_left = 0;
  nc->ringing_duration_ticks_left = 0;
  nc->display_blink_ticks_left = 0;
  nc->detoxification_idx = 0;

  nc->display_blink_interval_ms_ticks = nixie_calculate_display_blink_interval_ms_ticks(NIXIE_DISPLAY_BLINK_INTERVAL_MS_VALUE);

  nc->display_output_hv = NIXIE_DISPLAY_OUTPUT_HV_TYPE;
  #if (NIXIE_EEPROM_PRESENT == 1)
  uint8_t init_array[NIXIE_EEPROM_INIT_ARRAY_SIZE];
  if (nixie_read_eeprom(NIXIE_EEPROM_DISPLAY_OUTPUT_HV_ADDR, init_array, NIXIE_EEPROM_DISPLAY_OUTPUT_HV_SIZE))
  {
    nc->display_output_hv = init_array[0];
  }
  // Not being able to read converter type is a critical failure.
  else
  {
    Error_Handler();
  }
  if (nixie_read_eeprom(NIXIE_EEPROM_YEAR_HUNDREDS_ADDR, init_array, NIXIE_EEPROM_YEAR_HUNDREDS_SIZE))
  {
    nc->year_hundreds = nixie_uint8_array_to_uint16(init_array);
  }
  // Not being able to read year hundreds is quite a severe failure, as there is no way of restoring the year hundreds value. 
  else
  {
    Error_Handler();
  }
  // Failing to read the other EEPROM data is not as severe. The user will have to reconfigure the settings, but the device will otherwise be functional.
  if (nixie_read_eeprom(NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_ADDR, init_array, NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_SIZE))
  {
    nc->led_blink_interval_ms_ticks = nixie_calculate_led_blink_interval_ms_ticks(nixie_uint8_array_to_uint32(init_array));
  }
  if (nixie_read_eeprom(NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_SIZE))
  {
    nc->time_display_duration_ms = nixie_uint8_array_to_uint32(init_array);
  }
  if (nixie_read_eeprom(NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_SIZE))
  {
    nc->date_display_duration_ms = nixie_uint8_array_to_uint32(init_array);
  }
  if (nixie_read_eeprom(NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_SIZE))
  {
    nc->alarm_ringing_duration_ms_ticks = nixie_calculate_ringing_duration_ms_ticks(nixie_uint8_array_to_uint32(init_array));
  }
  if (nixie_read_eeprom(NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_SIZE))
  {
    nc->timer_ringing_duration_ms_ticks = nixie_calculate_ringing_duration_ms_ticks(nixie_uint8_array_to_uint32(init_array));
  }
  if (nixie_read_eeprom(NIXIE_EEPROM_DATE_FORMAT_ADDR, init_array, NIXIE_EEPROM_DATE_FORMAT_SIZE))
  {
    nc->date_format = init_array[0];
  } 
  #endif

  nc->display_state = NIXIE_DISPLAY_STATE_CLOCK;
  if (nc->time_display_duration_ms)
  {
    nc->clock_state = NIXIE_CLOCK_STATE_TIME;
  }
  else if (nc->date_display_duration_ms)
  {
    nc->clock_state = NIXIE_CLOCK_STATE_DATE;
  }
  else
  {
    nc->clock_state = NIXIE_CLOCK_STATE_NONE;
  }
  if (nc->clock_state != NIXIE_CLOCK_STATE_NONE)
  {
    nc->led_state = NIXIE_LED_STATE_ON;
  }
  else
  {
    nc->led_state = NIXIE_LED_STATE_OFF;
  }
  nc->blank_state = NIXIE_BLANK_STATE_OFF;
  nc->ringing_state = NIXIE_RINGING_STATE_OFF;
}

void nixie_set_gatt_clock_datetime_buffer(NixieController * const nc, RTC_TimeTypeDef const * const time, RTC_DateTypeDef const * const date)
{
  nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_SECONDS_IDX] = time->Seconds;
  nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MINUTES_IDX] = time->Minutes;
  nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_HOURS_IDX] = time->Hours;
  nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_DATE_IDX] = date->Date;
  nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MONTH_IDX] = date->Month;

  uint16_t year = date->Year + nc->year_hundreds * 100;

  nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_YEAR_MSB_IDX] = year >> CHAR_BIT;
  nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_YEAR_LSB_IDX] = year & 0xFF;
}

void nixie_update_rtc_year_overflow(NixieController * const nc, uint8_t current_year)
{
  if (!nc)
  {
    return;
  }
  if (nc->prev_rtc_year > current_year)
  {
    ++(nc->year_hundreds);
#if (NIXIE_EEPROM_PRESENT == 1)
    uint8_t array[NIXIE_UINT16_SIZE];
    nixie_uint16_to_uint8_array(array, nc->year_hundreds);
    if (!nixie_write_eeprom(NIXIE_EEPROM_YEAR_HUNDREDS_ADDR, array, NIXIE_EEPROM_YEAR_HUNDREDS_SIZE))
    {
      Error_Handler();
    }
#endif 
  }
  nc->prev_rtc_year = current_year;
}

bool nixie_is_datetime_update_needed(NixieController const * const nc, RTC_TimeTypeDef const * const time, RTC_DateTypeDef const * const date)
{
  if (!nc || !date || !time)
  {
    return false;
  }
  uint16_t year = ((nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_YEAR_MSB_IDX] << CHAR_BIT) | nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_YEAR_LSB_IDX]) % 100;
  if (nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_SECONDS_IDX] != time->Seconds || nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MINUTES_IDX] != time->Minutes
      || nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_HOURS_IDX] != time->Hours || nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_DATE_IDX] != date->Date
      || nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MONTH_IDX] != date->Month || year != date->Year)
  {
    return true;
  }
  return false;
}

void nixie_set_alarm_time(NixieController * const nc, NixieAlarmNumber alarm_number, NixieAlarmTime const * const time)
{
  if (!nc || !time)
  {
    return;
  }
  if (!nixie_compare_nixiealarmtime(&(nc->alarms_info[alarm_number].time), time))
  {
    return;
  }
  if (nc->alarms_info[alarm_number].status == NIXIE_ALARM_ON)
  {
    nixie_remove_alarm(nc, &(nc->alarms_info[alarm_number].time));
    nixie_add_alarm(nc, time);
    nixie_reschedule_alarm(nc, true);
  }
  nc->alarms_info[alarm_number].time = *time;
}

void nixie_set_alarm_status(NixieController * const nc, NixieAlarmNumber alarm_number, uint8_t status)
{
  if (!nc)
  {
    return;
  }
  if (nc->alarms_info[alarm_number].status == status)
  {
    return;
  }
  nc->alarms_info[alarm_number].status = status;
  if (nc->alarms_info[alarm_number].status == NIXIE_ALARM_ON)
  {
    nixie_add_alarm(nc, &(nc->alarms_info[alarm_number].time));
    nixie_reschedule_alarm(nc, true);
  }
  else{
    nixie_remove_alarm(nc, &(nc->alarms_info[alarm_number].time));
    nixie_reschedule_alarm(nc, true);
  }
}

bool nixie_is_valid_alarm_time(uint8_t const payload[const CUSTOM_STM_DATETIME_CHAR_SIZE])
{
  if (payload[NIXIE_DATETIME_YEAR_MSB_IDX] != 0 || payload[NIXIE_DATETIME_YEAR_LSB_IDX] != 0 || payload[NIXIE_DATETIME_MONTH_IDX] != 0
      || payload[NIXIE_DATETIME_DATE_IDX] != 0 || payload[NIXIE_DATETIME_HOURS_IDX] > 23 || payload[NIXIE_DATETIME_MINUTES_IDX] > 59
      || payload[NIXIE_DATETIME_SECONDS_IDX] > 59)
  {
    return false;
  }
  return true;
}

bool nixie_is_valid_alarm_status(uint8_t alarm_status)
{
  switch (alarm_status)
  {
    case NIXIE_ALARM_OFF:
      return true;
      break;
    case NIXIE_ALARM_ON:
      return true;
      break;
    default:
      return false;
      break;
  }
}

void nixie_init_rtc_alarms(void)
{
  if (HAL_RTC_DeactivateAlarm(&NIXIE_RTC_HANDLE, RTC_ALARM_A) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RTC_ALARM_CLEAR_FLAG(&NIXIE_RTC_HANDLE, RTC_FLAG_ALRAF);
  if (HAL_RTC_DeactivateAlarm(&NIXIE_RTC_HANDLE, RTC_ALARM_B) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RTC_ALARM_CLEAR_FLAG(&NIXIE_RTC_HANDLE, RTC_FLAG_ALRBF);
  if (HAL_RTC_RegisterCallback(&NIXIE_RTC_HANDLE, HAL_RTC_ALARM_A_EVENT_CB_ID, nixie_rtc_user_alarm_cb) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RTC_RegisterCallback(&NIXIE_RTC_HANDLE, HAL_RTC_ALARM_B_EVENT_CB_ID, nixie_rtc_hourly_alarm_cb) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure the hourly alarm.
  RTC_AlarmTypeDef alarm = {0};
  alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS;
  alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  alarm.AlarmDateWeekDay = 1;
  alarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm_IT(&NIXIE_RTC_HANDLE, &alarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
}

bool nixie_init_accelerometer(void)
{
  uint8_t addr;
  uint8_t data;
  uint8_t size;

  // Enter standby mode.
  addr = NIXIE_ACCLRM_POWER_CTL_ADDR;
  data = NIXIE_ACCLRM_POWER_CTL_INIT_MODE_VALUE;
  size = NIXIE_ACCLRM_POWER_CTL_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  // Initialize the registers.
  addr = NIXIE_ACCLRM_THRESH_TAP_ADDR;
  data = NIXIE_ACCLRM_THRESH_ACT_VALUE;
  size = NIXIE_ACCLRM_THRESH_TAP_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_OFSX_ADDR;
  data = NIXIE_ACCLRM_OFSX_VALUE;
  size = NIXIE_ACCLRM_OFSX_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_OFSY_ADDR;
  data = NIXIE_ACCLRM_OFSY_VALUE;
  size = NIXIE_ACCLRM_OFSY_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_OFSZ_ADDR;
  data = NIXIE_ACCLRM_OFSZ_VALUE;
  size = NIXIE_ACCLRM_OFSZ_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_DUR_ADDR;
  data = NIXIE_ACCLRM_DUR_VALUE;
  size = NIXIE_ACCLRM_DUR_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_LATENT_ADDR;
  data = NIXIE_ACCLRM_LATENT_VALUE;
  size = NIXIE_ACCLRM_LATENT_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_WINDOW_ADDR;
  data = NIXIE_ACCLRM_WINDOW_VALUE;
  size = NIXIE_ACCLRM_WINDOW_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_THRESH_ACT_ADDR;
  data = NIXIE_ACCLRM_THRESH_ACT_VALUE;
  size = NIXIE_ACCLRM_THRESH_ACT_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_THRESH_INACT_ADDR;
  data = NIXIE_ACCLRM_THRESH_INACT_VALUE;
  size = NIXIE_ACCLRM_THRESH_INACT_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_TIME_INACT_ADDR;
  data = NIXIE_ACCLRM_TIME_INACT_VALUE;
  size = NIXIE_ACCLRM_TIME_INACT_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_ACT_INACT_CTL_ADDR;
  data = NIXIE_ACCLRM_ACT_INACT_CTL_VALUE;
  size = NIXIE_ACCLRM_ACT_INACT_CTL_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_THRESH_FF_ADDR;
  data = NIXIE_ACCLRM_THRESH_FF_VALUE;
  size = NIXIE_ACCLRM_THRESH_FF_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_TIME_FF_ADDR;
  data = NIXIE_ACCLRM_TIME_FF_VALUE;
  size = NIXIE_ACCLRM_TIME_FF_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_TAP_AXES_ADDR;
  data = NIXIE_ACCLRM_TAP_AXES_VALUE;
  size = NIXIE_ACCLRM_TAP_AXES_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_BW_RATE_ADDR;
  data = NIXIE_ACCLRM_BW_RATE_VALUE;
  size = NIXIE_ACCLRM_BW_RATE_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_INT_ENABLE_ADDR;
  data = NIXIE_ACCLRM_INT_ENABLE_VALUE;
  size = NIXIE_ACCLRM_INT_ENABLE_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_INT_MAP_ADDR;
  data = NIXIE_ACCLRM_INT_MAP_VALUE;
  size = NIXIE_ACCLRM_INT_MAP_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_DATA_FORMAT_ADDR;
  data = NIXIE_ACCLRM_DATA_FORMAT_VALUE;
  size = NIXIE_ACCLRM_DATA_FORMAT_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  addr = NIXIE_ACCLRM_FIFO_CTL_ADDR;
  data = NIXIE_ACCLRM_FIFO_CTL_VALUE;
  size = NIXIE_ACCLRM_FIFO_CTL_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  // Initiate power control register as last, as not being in standby mode can cause undesired behavior.
  addr = NIXIE_ACCLRM_POWER_CTL_ADDR;
  data = NIXIE_ACCLRM_POWER_CTL_VALUE;
  size = NIXIE_ACCLRM_POWER_CTL_SIZE;
  if (!nixie_write_accelerometer(addr, &data, size))
  {
    return false;
  }
  return true;
}

bool nixie_write_accelerometer(uint16_t data_addr, uint8_t * const data, uint16_t data_size)
{
  if (!data_size)
  {
    return true;
  }
  if (!data)
  {
    return false;
  }
  if (HAL_I2C_Mem_Write(&NIXIE_I2C_HANDLE, NIXIE_ACCLRM_I2C_ADDR, data_addr, NIXIE_ACCLRM_MEMADD_SIZE, data, data_size, NIXIE_ACCLRM_TIMEOUT) != HAL_OK)
  {
    return false;
  }
  return true;
}

bool nixie_read_accelerometer(uint16_t data_addr, uint8_t * const data, uint16_t data_size)
{
  if (!data_size)
  {
    return true;
  }
  if (!data)
  {
    return false;
  }
  if (HAL_I2C_Mem_Read(&NIXIE_I2C_HANDLE, NIXIE_ACCLRM_I2C_ADDR, data_addr, NIXIE_ACCLRM_MEMADD_SIZE, data, data_size, NIXIE_ACCLRM_TIMEOUT) != HAL_OK)
  {
    return false;
  }
  return true;
}

bool nixie_init_eeprom(void)
{
  uint8_t init_array[NIXIE_EEPROM_INIT_ARRAY_SIZE];
  init_array[0] = NIXIE_DISPLAY_OUTPUT_HV_TYPE;
  if (!nixie_write_eeprom(NIXIE_EEPROM_DISPLAY_OUTPUT_HV_ADDR, init_array, NIXIE_EEPROM_DISPLAY_OUTPUT_HV_SIZE))
  {
    return false;
  }
  nixie_uint16_to_uint8_array(init_array, NIXIE_YEAR_HUNDREDS_DEFAULT_VALUE);
  if (!nixie_write_eeprom(NIXIE_EEPROM_YEAR_HUNDREDS_ADDR, init_array, NIXIE_EEPROM_YEAR_HUNDREDS_SIZE))
  {
    return false;
  }
  nixie_uint32_to_uint8_array(init_array, NIXIE_LED_BLINK_INTERVAL_DEFAULT_VALUE);
  if (!nixie_write_eeprom(NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_ADDR, init_array, NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_SIZE))
  {
    return false;
  }
  nixie_uint32_to_uint8_array(init_array, NIXIE_TIME_DISPLAY_DURATION_DEFAULT_VALUE);
  if (!nixie_write_eeprom(NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_SIZE))
  {
    return false;
  }
  nixie_uint32_to_uint8_array(init_array, NIXIE_DATE_DISPLAY_DURATION_DEFAULT_VALUE);
  if (!nixie_write_eeprom(NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_SIZE))
  {
    return false;
  }
  nixie_uint32_to_uint8_array(init_array, NIXIE_ALARM_RINGING_DURATION_DEFAULT_VALUE);
  if (!nixie_write_eeprom(NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_SIZE))
  {
    return false;
  }
  nixie_uint32_to_uint8_array(init_array, NIXIE_TIMER_RINGING_DURATION_DEFAULT_VALUE);
  if (!nixie_write_eeprom(NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_ADDR, init_array, NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_SIZE))
  {
    return false;
  }
  init_array[0] = NIXIE_DATE_FORMAT_DEFAULT_VALUE;
  if (!nixie_write_eeprom(NIXIE_EEPROM_DATE_FORMAT_ADDR, init_array, NIXIE_EEPROM_DATE_FORMAT_SIZE))
  {
    return false;
  } 
  return true;
}

bool nixie_write_eeprom(uint16_t data_addr, uint8_t * const data, uint16_t data_size)
{
  if (!data_size)
  {
    return true;
  }
  if (!data)
  {
    return false;
  }
  for (unsigned i = 0; i < data_size; ++i)
  {
    if (HAL_I2C_Mem_Write(&NIXIE_I2C_HANDLE, NIXIE_EEPROM_I2C_ADDR, data_addr + i, NIXIE_EEPROM_MEMADD_SIZE, &(data[i]), 1, NIXIE_EEPROM_TIMEOUT) != HAL_OK)
    {
      return false;
    }
    HAL_Delay(NIXIE_EEPROM_WRITE_CYCLE_MS);
  }
  return true;
}

bool nixie_read_eeprom(uint16_t data_addr, uint8_t * const data, uint16_t data_size)
{
  if (!data_size)
  {
    return true;
  }
  if (!data)
  {
    return false;
  }
  if (HAL_I2C_Mem_Read(&NIXIE_I2C_HANDLE, NIXIE_EEPROM_I2C_ADDR, data_addr, NIXIE_EEPROM_MEMADD_SIZE, data, data_size, NIXIE_EEPROM_TIMEOUT) != HAL_OK)
  {
    return false;
  }
  return true;
}

void nixie_transmit_display_spi_data(uint64_t data)
{
  uint16_t tmp[NIXIE_DISPLAY_SPI_ARRAY_SIZE];

  HAL_GPIO_WritePin(NIXIE_LATCH_PIN_PORT, NIXIE_LATCH_PIN_NUMBER, NIXIE_LATCH_PIN_LATCHED);
  for (int i = 0; i < NIXIE_DISPLAY_SPI_ARRAY_SIZE; ++i)
  {
    tmp[i] = data >> ((NIXIE_DISPLAY_SPI_ARRAY_SIZE - 1 - i) * NIXIE_DISPLAY_SPI_DATA_BITSIZE);
  }
  if (HAL_SPI_Transmit_IT(&NIXIE_DISPLAY_SPI_HANDLE, (uint8_t*) tmp, NIXIE_DISPLAY_SPI_ARRAY_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
}

uint64_t nixie_calculate_display_data(NixieController * const nc)
{
  uint8_t array[NIXIE_OPT_NUM];
  NixieSetDigitArrayStatus empty = NIXIE_DIGIT_ARRAY_NOT_EMPTY;

  switch (nc->display_state)
  {
    case NIXIE_DISPLAY_STATE_CLOCK:
      empty = nixie_set_digit_array_clock(nc, array);
      break;
    case NIXIE_DISPLAY_STATE_TIMER:
      nixie_set_digit_array_seconds(array, nc->timer_time_s);
      break;
    case NIXIE_DISPLAY_STATE_STOPWATCH:
      nixie_set_digit_array_seconds(array, nc->stopwatch_time_s);
      break;
    case NIXIE_DISPLAY_STATE_DETOX:
      array[0] = nc->detoxification_idx % NIXIE_DIGITS_NUM;
      for (int i = 1; i < NIXIE_OPT_NUM; ++i)
      {
        array[i] = array[0];
      }
    default:
      break;
  }
  if (nc->display_output_hv == NIXIE_DISPLAY_OUTPUT_HV5623)
  {
    return (empty == NIXIE_DIGIT_ARRAY_NOT_EMPTY)? nixie_calculate_display_data_hv5623(array): NIXIE_EMPTY_DISPLAY_HV5623;
  }
  // HV5523
  else
  {
    return (empty == NIXIE_DIGIT_ARRAY_NOT_EMPTY)? nixie_calculate_display_data_hv5523(array): NIXIE_EMPTY_DISPLAY_HV5523;
  }
}

void nixie_update_blank_pin(NixieController const * const nc)
{
  if (nc->ringing_state != NIXIE_RINGING_STATE_OFF && nc->blank_state == NIXIE_BLANK_STATE_ON)
  {
    HAL_GPIO_WritePin(NIXIE_BLANK_PIN_PORT, NIXIE_BLANK_PIN_NUMBER, NIXIE_BLANK_PIN_BLANK);
  }
  else
  {
    HAL_GPIO_WritePin(NIXIE_BLANK_PIN_PORT, NIXIE_BLANK_PIN_NUMBER, NIXIE_BLANK_PIN_NOT_BLANK);
  }
}

void nixie_reschedule_alarm(NixieController * const nc, bool alarms_changed)
{
  if (!nc)
  {
    return;
  }
  if (!nc->active_alarms_size)
  {
    if (HAL_RTC_DeactivateAlarm(&NIXIE_RTC_HANDLE, RTC_ALARM_A) != HAL_OK)
    {
      Error_Handler();
    }
    return;
  }
  RTC_AlarmTypeDef alarm = {0};
  alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  alarm.AlarmDateWeekDay = 1;
  alarm.Alarm = RTC_ALARM_A;
  if (alarms_changed)
  {
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    int idx;
    NixieAlarmTime tmp;
    // The next alarm to be scheduled should be the smallest alarm time that is bigger than current time.
    // If the current time is bigger than all alarms, the closest alarm is the smallest one.
    for (idx = 0; idx < nc->active_alarms_size; ++idx)
    {
      if (HAL_RTC_GetTime(&NIXIE_RTC_HANDLE, &time, RTC_FORMAT_BIN) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_RTC_GetDate(&NIXIE_RTC_HANDLE, &date, RTC_FORMAT_BIN) != HAL_OK)
      {
        Error_Handler();
      }
      tmp.seconds = time.Seconds;
      tmp.minutes = time.Minutes;
      tmp.hours = time.Hours;
      if (nixie_compare_nixiealarmtime(&tmp, &(nc->active_alarms[idx].time)) == -1)
      {
        break;
      }
    }
    nc->active_alarms_scheduled_idx = idx % nc->active_alarms_size;
  }
  else
  {
    ++(nc->active_alarms_scheduled_idx);
    nc->active_alarms_scheduled_idx %= nc->active_alarms_size;
  }
  alarm.AlarmTime.Hours = nc->active_alarms[nc->active_alarms_scheduled_idx].time.hours;
  alarm.AlarmTime.Minutes = nc->active_alarms[nc->active_alarms_scheduled_idx].time.minutes;
  alarm.AlarmTime.Seconds = nc->active_alarms[nc->active_alarms_scheduled_idx].time.seconds;
  if (HAL_RTC_SetAlarm_IT(&NIXIE_RTC_HANDLE, &alarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
}

void nixie_start_stopwatch_mode(NixieController * const nc)
{
  if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  nc->display_state = NIXIE_DISPLAY_STATE_STOPWATCH;
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
  NIXIE_DISPLAY_TIM_HANDLE.Init.Prescaler = 0;
  NIXIE_DISPLAY_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
  NIXIE_DISPLAY_TIM_HANDLE.Init.RepetitionCounter = 0;
  // 1 second's worth of ticks always fits into period register.
  NIXIE_DISPLAY_TIM_HANDLE.Init.Period = nixie_calculate_tim_ticks_ms(&NIXIE_DISPLAY_TIM_HANDLE, 1000) - 1;
  NIXIE_DISPLAY_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  NIXIE_DISPLAY_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_CLEAR_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
  __HAL_TIM_SET_COUNTER(&NIXIE_DISPLAY_TIM_HANDLE, nc->stopwatch_tim_cnt);
  if (HAL_TIM_RegisterCallback(&NIXIE_DISPLAY_TIM_HANDLE, HAL_TIM_PERIOD_ELAPSED_CB_ID, nixie_display_timer_stopwatch_cb) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();  
  }
}

void nixie_stop_stopwatch_mode(NixieController * const nc)
{
  nc->stopwatch_time_s = 0;
  nc->stopwatch_tim_cnt = 0;
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_STOPWATCH_TIME_ID, CFG_SCH_PRIO_1);
  if (nc->display_state != NIXIE_DISPLAY_STATE_STOPWATCH)
  {
    return;
  }
  if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (nc->timer_status != NIXIE_TIMER_PAUSED)
  {
    nc->display_state = NIXIE_DISPLAY_STATE_CLOCK;
    nixie_update_clock_timer(nc);
  }
  else
  {
    nc->display_state = NIXIE_DISPLAY_STATE_TIMER;
  }
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
}

void nixie_pause_stopwatch_mode(NixieController * const nc)
{
  if (nc->display_state != NIXIE_DISPLAY_STATE_STOPWATCH)
  {
    return;
  }
  if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  nc->stopwatch_tim_cnt = __HAL_TIM_GET_COUNTER(&NIXIE_DISPLAY_TIM_HANDLE);
}

void nixie_start_timer_mode(NixieController * const nc)
{
  if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  nc->display_state = NIXIE_DISPLAY_STATE_TIMER;
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
  if (!nc->timer_time_s)
  {
    NIXIE_CONTROLLER_HANDLE.timer_status = NIXIE_TIMER_OFF;
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_STATUS_ID, CFG_SCH_PRIO_1);
    nixie_start_ringing(nc, NIXIE_RINGING_SOURCE_TIMER);
    return;
  }
  NIXIE_DISPLAY_TIM_HANDLE.Init.Prescaler = 0;
  NIXIE_DISPLAY_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
  NIXIE_DISPLAY_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  NIXIE_DISPLAY_TIM_HANDLE.Init.RepetitionCounter = 0;
  // 1 second's worth of ticks always fits into period register.
  NIXIE_DISPLAY_TIM_HANDLE.Init.Period = nixie_calculate_tim_ticks_ms(&NIXIE_DISPLAY_TIM_HANDLE, 1000) - 1;
  NIXIE_DISPLAY_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_CLEAR_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
  __HAL_TIM_SET_COUNTER(&NIXIE_DISPLAY_TIM_HANDLE, nc->timer_tim_cnt);
  if (HAL_TIM_RegisterCallback(&NIXIE_DISPLAY_TIM_HANDLE, HAL_TIM_PERIOD_ELAPSED_CB_ID, nixie_display_timer_timer_cb) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();  
  }
}

void nixie_set_timer_time_s(NixieController * const nc, uint32_t time_s)
{
  if (nc->timer_status == NIXIE_TIMER_ON)
  {
    if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
    {
      Error_Handler();
    }
    nc->timer_time_s = time_s;
    nc->timer_tim_cnt = 0;
    nixie_start_timer_mode(nc);
  }
  else
  {
    nc->timer_time_s = time_s;
    nc->timer_tim_cnt = 0;
  }
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_TIME_ID, CFG_SCH_PRIO_1);
}

void nixie_stop_timer_mode(NixieController * const nc)
{
  nc->timer_time_s = 0;
  nc->timer_tim_cnt = 0;
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_NOTIFY_GATT_TIMER_TIME_ID, CFG_SCH_PRIO_1);
  if (nc->display_state != NIXIE_DISPLAY_STATE_TIMER)
  {
    return;
  }
  if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (nc->display_state != NIXIE_DISPLAY_STATE_TIMER)
  {
    return;
  }
  if (nc->stopwatch_status != NIXIE_STOPWATCH_PAUSED)
  {
    nc->display_state = NIXIE_DISPLAY_STATE_CLOCK;
    nixie_update_clock_timer(nc);
  }
  else
  {
    nc->display_state = NIXIE_DISPLAY_STATE_STOPWATCH;
  }
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
}

void nixie_pause_timer_mode(NixieController * const nc)
{
  if (nc->display_state != NIXIE_DISPLAY_STATE_TIMER)
  {
    return;
  }
  if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  nc->timer_tim_cnt = __HAL_TIM_GET_COUNTER(&NIXIE_DISPLAY_TIM_HANDLE);
}

void nixie_update_clock_timer(NixieController * const nc)
{
  if (!nc)
  {
    return;
  }
  // We need to show both time and date, check if timer needs to be set up.
  if (nc->time_display_duration_ms && nc->date_display_duration_ms)
  {
    // If other functions aren't using the display timer, restart it.
    __HAL_TIM_DISABLE_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
    if (nc->display_state == NIXIE_DISPLAY_STATE_CLOCK)
    {
      if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE))
      {
        Error_Handler();
      }
      nc->clock_state = NIXIE_CLOCK_STATE_TIME;
      NIXIE_DISPLAY_TIM_HANDLE.Init.Prescaler = nixie_calculate_tim_ticks_ms(&NIXIE_DISPLAY_TIM_HANDLE, 1) - 1;
      NIXIE_DISPLAY_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
      NIXIE_DISPLAY_TIM_HANDLE.Init.RepetitionCounter = 0;
      NIXIE_DISPLAY_TIM_HANDLE.Init.Period = nc->time_display_duration_ms - 1;
      NIXIE_DISPLAY_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      NIXIE_DISPLAY_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
      if (HAL_TIM_Base_Init(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
      {
        Error_Handler();
      }
      // Init function generates an update event. We do not want the interrupt to happen for this particular event.
      __HAL_TIM_CLEAR_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
      if (HAL_TIM_RegisterCallback(&NIXIE_DISPLAY_TIM_HANDLE, HAL_TIM_PERIOD_ELAPSED_CB_ID, nixie_display_timer_clock_cb) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_TIM_Base_Start_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
      {
        Error_Handler();  
      }
      UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
      UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
    }
    else
    {
      __HAL_TIM_ENABLE_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
    }
  }
  // Showing only one of the parts or neither of them, don't need to waste CPU time with timer handling.
  else if (nc->time_display_duration_ms)
  {
    // Stop the display timer if it is handling clock data change.
    __HAL_TIM_DISABLE_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
    if (nc->display_state == NIXIE_DISPLAY_STATE_CLOCK)
    {
      if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
      {
        Error_Handler();
      }
    }
    else
    {
      __HAL_TIM_ENABLE_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
    }
    nc->clock_state = NIXIE_CLOCK_STATE_TIME;
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
  }
  else if (nc->date_display_duration_ms)
  {
    // Stop the display timer if it is handling clock data change.
    __HAL_TIM_DISABLE_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
    if (nc->display_state == NIXIE_DISPLAY_STATE_CLOCK)
    {
      if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
      {
        Error_Handler();
      }
    }
    else
    {
      __HAL_TIM_ENABLE_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
    }
    nc->clock_state = NIXIE_CLOCK_STATE_DATE;
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
  }
  else
  {
    nc->clock_state = NIXIE_CLOCK_STATE_NONE;
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_LED_ID, CFG_SCH_PRIO_1);
  }
}

void nixie_update_led_timer(NixieController * const nc)
{
  if (!nc)
  {
    return;
  }
  nixie_update_lptim_timer(nc, NIXIE_UPDATE_LED_BLINK_LPTIM);
}

void nixie_start_ringing(NixieController * const nc, NixieRingingSource source)
{
  switch (source)
  {
    case NIXIE_RINGING_SOURCE_ALARM:
      if (!nc->alarm_ringing_duration_ms_ticks)
      {
        return;
      }
      nixie_start_pwm_timer();
      nixie_update_lptim_timer(nc, NIXIE_UPDATE_ALARM_RINGING_LPTIM);
      nc->ringing_state = NIXIE_RINGING_STATE_ALARM;
      if (!nc->led_blink_interval_ms_ticks)
      {
        nixie_update_lptim_timer(nc, NIXIE_UPDATE_DISPLAY_BLINK_LPTIM);
      }
      break;
    case NIXIE_RINGING_SOURCE_TIMER:
      if (!nc->timer_ringing_duration_ms_ticks)
      {
        nc->display_state = NIXIE_DISPLAY_STATE_CLOCK;
        UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
        return;
      }
      nixie_start_pwm_timer();
      nixie_update_lptim_timer(nc, NIXIE_UPDATE_TIMER_RINGING_LPTIM);
      nc->ringing_state = NIXIE_RINGING_STATE_TIMER;
      if (!nc->led_blink_interval_ms_ticks)
      {
        nixie_update_lptim_timer(nc, NIXIE_UPDATE_DISPLAY_BLINK_LPTIM);
      }
      break;
    default:
      break;
  }
}

void nixie_stop_ringing(NixieController * const nc)
{
  if (nc->ringing_state == NIXIE_RINGING_STATE_OFF)
  {
    return;
  }
  nixie_stop_pwm_timer();
  if (HAL_LPTIM_Counter_Stop_IT(&NIXIE_RINGING_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (!nc->led_blink_interval_ms_ticks)
  {
    if (HAL_LPTIM_Counter_Stop_IT(&NIXIE_BLANK_TIM_HANDLE) != HAL_OK)
    {
      Error_Handler();
    }
  }
  nc->blank_state = NIXIE_BLANK_STATE_OFF;
  nixie_update_blank_pin(nc);
  if (nc->ringing_state == NIXIE_RINGING_STATE_TIMER)
  {
    nc->display_state = NIXIE_DISPLAY_STATE_CLOCK;
    nixie_update_clock_timer(nc);
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  }
  nc->ringing_state = NIXIE_RINGING_STATE_OFF;
}

void nixie_process_accelerometer_activity(NixieController * const nc)
{
  nixie_stop_ringing(nc);
  // Unlatch interrupt by reading INT_SOURCE or DATAX/DATAY/DATAZ.
  uint8_t tmp;
  nixie_read_accelerometer(NIXIE_ACCLRM_INT_SOURCE_ADDR, &tmp, NIXIE_ACCLRM_INT_SOURCE_SIZE);
}

void nixie_update_dst(void)
{
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  uint32_t expected_state;

  if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  expected_state = nixie_calculate_dst_state(&time, &date, NIXIE_DST_CALCULATION_UPDATE); 
  uint32_t real_state = HAL_RTC_DST_ReadStoreOperation(&NIXIE_RTC_HANDLE);

  if (real_state != expected_state)
  {
    if (real_state == NIXIE_RTC_DST_STORE_STANDARD_TIME)
    {
      HAL_RTC_DST_SetStoreOperation(&NIXIE_RTC_HANDLE);
      HAL_RTC_DST_Add1Hour(&NIXIE_RTC_HANDLE);
    }
    else
    {
      HAL_RTC_DST_ClearStoreOperation(&NIXIE_RTC_HANDLE);
      HAL_RTC_DST_Sub1Hour(&NIXIE_RTC_HANDLE);
    }
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
  }
}

void nixie_update_gatt_clock_datetime(NixieController const * const nc)
{
  if (Custom_STM_App_Update_Char(CUSTOM_STM_CLOCK_DATETIME, nc->gatt_clock_datetime_buffer) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_alarm_status(NixieController const * const nc, NixieAlarmNumber alarm_number)
{
  Custom_STM_Char_Opcode_t opcode;
  switch (alarm_number)
  {
    case NIXIE_ALARM1:
      opcode = CUSTOM_STM_ALARM1_STATUS;
      break;
    case NIXIE_ALARM2:
      opcode = CUSTOM_STM_ALARM2_STATUS;
      break;
    case NIXIE_ALARM3:
      opcode = CUSTOM_STM_ALARM3_STATUS;
      break;
    case NIXIE_ALARM4:
      opcode = CUSTOM_STM_ALARM4_STATUS;
      break;
    case NIXIE_ALARM5:
      opcode = CUSTOM_STM_ALARM5_STATUS;
      break;
    case NIXIE_ALARM6:
      opcode = CUSTOM_STM_ALARM6_STATUS;
      break;
    case NIXIE_ALARM7:
      opcode = CUSTOM_STM_ALARM7_STATUS;
      break;
    case NIXIE_ALARM8:
      opcode = CUSTOM_STM_ALARM8_STATUS;
      break;
    case NIXIE_ALARM9:
      opcode = CUSTOM_STM_ALARM9_STATUS;
      break;
    case NIXIE_ALARM10:
      opcode = CUSTOM_STM_ALARM10_STATUS;
      break;
    default:
      return;
      break;
  }
  if (Custom_STM_App_Update_Char(opcode, &(nc->alarms_info[alarm_number].status)) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_alarm_time(NixieController const * const nc, NixieAlarmNumber alarm_number)
{
  Custom_STM_Char_Opcode_t opcode;
  switch (alarm_number)
  {
    case NIXIE_ALARM1:
      opcode = CUSTOM_STM_ALARM1_TIME;
      break;
    case NIXIE_ALARM2:
      opcode = CUSTOM_STM_ALARM2_TIME;
      break;
    case NIXIE_ALARM3:
      opcode = CUSTOM_STM_ALARM3_TIME;
      break;
    case NIXIE_ALARM4:
      opcode = CUSTOM_STM_ALARM4_TIME;
      break;
    case NIXIE_ALARM5:
      opcode = CUSTOM_STM_ALARM5_TIME;
      break;
    case NIXIE_ALARM6:
      opcode = CUSTOM_STM_ALARM6_TIME;
      break;
    case NIXIE_ALARM7:
      opcode = CUSTOM_STM_ALARM7_TIME;
      break;
    case NIXIE_ALARM8:
      opcode = CUSTOM_STM_ALARM8_TIME;
      break;
    case NIXIE_ALARM9:
      opcode = CUSTOM_STM_ALARM9_TIME;
      break;
    case NIXIE_ALARM10:
      opcode = CUSTOM_STM_ALARM10_TIME;
      break;
    default:
      return;
      break;
  }
  uint8_t array[CUSTOM_STM_DATETIME_CHAR_SIZE] = {0};
  array[NIXIE_DATETIME_HOURS_IDX] = nc->alarms_info[alarm_number].time.hours;
  array[NIXIE_DATETIME_MINUTES_IDX] = nc->alarms_info[alarm_number].time.minutes;
  array[NIXIE_DATETIME_SECONDS_IDX] = nc->alarms_info[alarm_number].time.seconds;
  if (Custom_STM_App_Update_Char(opcode, array) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_stopwatch_time_s(NixieController const * const nc)
{
  uint8_t tmp[NIXIE_UINT32_SIZE];
  nixie_uint32_to_uint8_array(tmp, nc->stopwatch_time_s);
  if (Custom_STM_App_Update_Char(CUSTOM_STM_STOPWATCH_TIME_S, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_stopwatch_status(NixieController const * const nc)
{
  if (Custom_STM_App_Update_Char(CUSTOM_STM_STOPWATCH_STATUS, &(nc->stopwatch_status)) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_timer_time_s(NixieController const * const nc)
{
  uint8_t tmp[NIXIE_UINT32_SIZE];
  nixie_uint32_to_uint8_array(tmp, nc->timer_time_s);
  if (Custom_STM_App_Update_Char(CUSTOM_STM_TIMER_TIME_S, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_timer_status(NixieController const * const nc)
{
  if (Custom_STM_App_Update_Char(CUSTOM_STM_TIMER_STATUS, &(nc->timer_status)) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_settings_led_blink_interval_ms(NixieController const * const nc)
{
  uint8_t tmp[CUSTOM_STM_UINT32_CHAR_SIZE];
  nixie_uint32_to_uint8_array(tmp, nixie_calculate_period_lptim_ms(&NIXIE_LED_TIM_HANDLE, nc->led_blink_interval_ms_ticks));
  if (Custom_STM_App_Update_Char(CUSTOM_STM_SETTINGS_LED_BLINK_INTERVAL_MS, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_settings_time_display_duration_ms(NixieController const * const nc)
{
  uint8_t tmp[CUSTOM_STM_UINT32_CHAR_SIZE];
  nixie_uint32_to_uint8_array(tmp, nc->time_display_duration_ms);
  if (Custom_STM_App_Update_Char(CUSTOM_STM_SETTINGS_TIME_DISPLAY_DURATION_MS, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_settings_date_display_duration_ms(NixieController const * const nc)
{
  uint8_t tmp[CUSTOM_STM_UINT32_CHAR_SIZE];
  nixie_uint32_to_uint8_array(tmp, nc->date_display_duration_ms);
  if (Custom_STM_App_Update_Char(CUSTOM_STM_SETTINGS_DATE_DISPLAY_DURATION_MS, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_settings_alarm_ringing_duration_ms(NixieController const * const nc)
{
  uint8_t tmp[CUSTOM_STM_UINT32_CHAR_SIZE];
  nixie_uint32_to_uint8_array(tmp, nixie_calculate_period_lptim_ms(&NIXIE_RINGING_TIM_HANDLE, nc->alarm_ringing_duration_ms_ticks));
  if (Custom_STM_App_Update_Char(CUSTOM_STM_SETTINGS_ALARM_RINGING_DURATION_MS, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_settings_timer_ringing_duration_ms(NixieController const * const nc)
{
  uint8_t tmp[CUSTOM_STM_UINT32_CHAR_SIZE];
  nixie_uint32_to_uint8_array(tmp, nixie_calculate_period_lptim_ms(&NIXIE_RINGING_TIM_HANDLE, nc->timer_ringing_duration_ms_ticks));
  if (Custom_STM_App_Update_Char(CUSTOM_STM_SETTINGS_TIMER_RINGING_DURATION_MS, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_settings_date_format(NixieController const * const nc)
{
  if (Custom_STM_App_Update_Char(CUSTOM_STM_SETTINGS_DATE_FORMAT, &(nc->date_format)) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_rtc_calibration_calp(void)
{
  uint8_t tmp = (uint8_t) LL_RTC_CAL_IsPulseInserted(NIXIE_RTC_HANDLE.Instance);
  if (Custom_STM_App_Update_Char(CUSTOM_STM_RTC_CALIBRATION_CALP, &tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}

void nixie_update_gatt_rtc_calibration_calm(void)
{
  uint8_t tmp[CUSTOM_STM_CALM_CHAR_SIZE];
  nixie_uint16_to_uint8_array(tmp, (uint16_t) LL_RTC_CAL_GetMinus(NIXIE_RTC_HANDLE.Instance));  
  if (Custom_STM_App_Update_Char(CUSTOM_STM_RTC_CALIBRATION_CALM, tmp) != BLE_STATUS_SUCCESS)
  {
    Error_Handler();
  }
}


bool nixie_is_valid_date_time(uint8_t const payload[const CUSTOM_STM_DATETIME_CHAR_SIZE])
{
  static unsigned int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  // Year doesn't need to be checked.
  if (payload[NIXIE_DATETIME_MONTH_IDX] > 12 || payload[NIXIE_DATETIME_HOURS_IDX] > 23 || payload[NIXIE_DATETIME_MINUTES_IDX] > 59
      || payload[NIXIE_DATETIME_SECONDS_IDX] > 59)
  {
    return false;
  }
  uint16_t year = (payload[NIXIE_DATETIME_YEAR_MSB_IDX] << CHAR_BIT) | payload[NIXIE_DATETIME_YEAR_LSB_IDX];
  uint8_t leap_day = (payload[NIXIE_DATETIME_MONTH_IDX] == 2) && !(year % 4) && (year % 100 || !(year % 400))? 1: 0;
  if (payload[NIXIE_DATETIME_DATE_IDX] > days_in_month[payload[NIXIE_DATETIME_MONTH_IDX] - 1] + leap_day)
  {
    return false;
  }
  return true;
}

bool nixie_is_valid_date_format(uint8_t payload)
{
  switch (payload)
  {
    case NIXIE_DATE_FORMAT_EU:
      return true;
      break;
    case NIXIE_DATE_FORMAT_US:
      return true;
      break;
    default:
      return false;
      break;
  }
}

bool nixie_is_valid_rtc_calibration_calp(uint8_t calp)
{
  if (calp == 1 || !calp)
  {
    return true;
  }
  return false;
}

void nixie_update_leds(NixieController * const nc)
{
  if (!nc)
  {
    return;
  }
  // LED state only needs to be managed manually if led blinking interval is 0.
  if (!nc->led_blink_interval_ms_ticks)
  {
    if (nc->display_state == NIXIE_DISPLAY_STATE_CLOCK && nc->clock_state == NIXIE_CLOCK_STATE_NONE)
    {
      nc->led_state = NIXIE_LED_STATE_OFF;
    }
    else
    {
      nc->led_state = NIXIE_LED_STATE_ON;
    }
  }
  // Update LED pins.
  if (nc->led_state == NIXIE_LED_STATE_ON)
  {
    if (nc->display_state != NIXIE_DISPLAY_STATE_CLOCK || nc->clock_state == NIXIE_CLOCK_STATE_TIME)
    {
      HAL_GPIO_WritePin(NIXIE_LED1_PIN_PORT, NIXIE_LED1_PIN_NUMBER, NIXIE_LED_PIN_ON);
      HAL_GPIO_WritePin(NIXIE_LED2_PIN_PORT, NIXIE_LED2_PIN_NUMBER, NIXIE_LED_PIN_ON);
      HAL_GPIO_WritePin(NIXIE_LED3_PIN_PORT, NIXIE_LED3_PIN_NUMBER, NIXIE_LED_PIN_ON);
      HAL_GPIO_WritePin(NIXIE_LED4_PIN_PORT, NIXIE_LED4_PIN_NUMBER, NIXIE_LED_PIN_ON);
    }
    else if (nc->clock_state == NIXIE_CLOCK_STATE_DATE)
    {
      HAL_GPIO_WritePin(NIXIE_LED1_PIN_PORT, NIXIE_LED1_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED2_PIN_PORT, NIXIE_LED2_PIN_NUMBER, NIXIE_LED_PIN_ON);
      HAL_GPIO_WritePin(NIXIE_LED3_PIN_PORT, NIXIE_LED3_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED4_PIN_PORT, NIXIE_LED4_PIN_NUMBER, NIXIE_LED_PIN_ON);
    }
    // NIXIE_CLOCK_NONE
    else
    {
      HAL_GPIO_WritePin(NIXIE_LED1_PIN_PORT, NIXIE_LED1_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED2_PIN_PORT, NIXIE_LED2_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED3_PIN_PORT, NIXIE_LED3_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED4_PIN_PORT, NIXIE_LED4_PIN_NUMBER, NIXIE_LED_PIN_OFF);
    }
  }
  else
  {
      HAL_GPIO_WritePin(NIXIE_LED1_PIN_PORT, NIXIE_LED1_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED2_PIN_PORT, NIXIE_LED2_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED3_PIN_PORT, NIXIE_LED3_PIN_NUMBER, NIXIE_LED_PIN_OFF);
      HAL_GPIO_WritePin(NIXIE_LED4_PIN_PORT, NIXIE_LED4_PIN_NUMBER, NIXIE_LED_PIN_OFF);
  }
}

void nixie_set_datetime_members(NixieController * const nc, const RTC_TimeTypeDef * const time, const RTC_DateTypeDef * const date, uint16_t year)
{
  uint32_t dst_state = nixie_calculate_dst_state(time, date, NIXIE_DST_CALCULATION_SET);
  if (dst_state == NIXIE_RTC_DST_STORE_STANDARD_TIME)
  {
    HAL_RTC_DST_ClearStoreOperation(&NIXIE_RTC_HANDLE);
  }
  else
  {
    HAL_RTC_DST_SetStoreOperation(&NIXIE_RTC_HANDLE);
  }
#if (NIXIE_EEPROM_PRESENT == 1)
  uint16_t prev_year_hundreds = nc->year_hundreds;
#endif
  nc->year_hundreds = year / 100;
 #if NIXIE_EEPROM_PRESENT == 1
  if (nc->year_hundreds != prev_year_hundreds)
  {
    uint8_t array[NIXIE_UINT16_SIZE];
    nixie_uint16_to_uint8_array(array, nc->year_hundreds);
    if (!nixie_write_eeprom(NIXIE_EEPROM_YEAR_HUNDREDS_ADDR, array, NIXIE_EEPROM_YEAR_HUNDREDS_SIZE))
    {
      Error_Handler();
    }    
  }
#endif 
  nc->prev_rtc_year = date->Year;
  nixie_set_gatt_clock_datetime_buffer(nc, time, date);  
}

void nixie_process_hourly_check(NixieController * const nc)
{
  nixie_update_dst();
  if (nc->display_state == NIXIE_DISPLAY_STATE_CLOCK)
  {
    nixie_detoxify(nc);
  }
}

void nixie_init_gatt_server(NixieController const * const nc)
{
  // Clock
  nixie_update_gatt_clock_datetime(nc);
  // Alarms
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM1);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM2);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM3);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM4);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM5);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM6);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM7);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM8);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM9);
  nixie_update_gatt_alarm_time(nc, NIXIE_ALARM10);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM1);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM2);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM3);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM4);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM5);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM6);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM7);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM8);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM9);
  nixie_update_gatt_alarm_status(nc, NIXIE_ALARM10);
  // Stopwatch and Timer
  nixie_update_gatt_stopwatch_time_s(nc);
  nixie_update_gatt_stopwatch_status(nc);
  nixie_update_gatt_timer_time_s(nc);
  nixie_update_gatt_timer_status(nc);
  // Settings
  nixie_update_gatt_settings_led_blink_interval_ms(nc);
  nixie_update_gatt_settings_time_display_duration_ms(nc);
  nixie_update_gatt_settings_date_display_duration_ms(nc);
  nixie_update_gatt_settings_alarm_ringing_duration_ms(nc);
  nixie_update_gatt_settings_timer_ringing_duration_ms(nc);
  nixie_update_gatt_settings_date_format(nc);
  // RTC Calibration
  nixie_update_gatt_rtc_calibration_calp();
  nixie_update_gatt_rtc_calibration_calm();
}

static uint64_t nixie_calculate_display_data_hv5523(uint8_t const array[const NIXIE_OPT_NUM])
{
  uint64_t rv;

  rv = 0x0;
  for (int i = 0; i < NIXIE_OPT_NUM; ++i)
  {
    if (array[i] > NIXIE_DIGITS_MAX_VALUE)
    {
      return 0x0;  
    }
    rv |= 0x1ULL << (nixie_digit_offset_hv5523[array[i]] + nixie_opt_offset_hv5523[i]);
  }
  // Differentiate between error and success by using bits that do not control any tube symbols. 
  if (!rv)
  {
    rv = NIXIE_EMPTY_DISPLAY_HV5523;
  }
  return rv;
}

static uint64_t nixie_calculate_display_data_hv5623(uint8_t const array[const NIXIE_OPT_NUM])
{
  uint64_t rv;

  rv = 0x0;
  for (int i = 0; i < NIXIE_OPT_NUM; ++i)
  {
    if (array[i] > NIXIE_DIGITS_MAX_VALUE){
      return 0x0;
    }
    rv |= 0x1ULL << (nixie_digit_offset_hv5623[array[i]] + nixie_opt_offset_hv5623[i]);
  }
  // Differentiate between error and success by using bits that do not control any tube symbols. 
  if (!rv)
  {
    rv = NIXIE_EMPTY_DISPLAY_HV5623;
  }
  return rv;
}

static void nixie_start_pwm_timer(void)
{
  uint64_t period_ticks = nixie_calculate_tim_ticks_ms(&NIXIE_PWM_TIM_HANDLE, 1000) / NIXIE_PWM_FREQUENCY_HZ;
  // PSC = period_ticks / (ARR_MAX + 1) + 1. To achieve the desired PSC, subtract 1 when adding to the register.
  NIXIE_PWM_TIM_HANDLE.Init.Prescaler = (uint16_t)(period_ticks / (NIXIE_PWM_TIM_ARR_MAX + 1));
  NIXIE_PWM_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
  NIXIE_PWM_TIM_HANDLE.Init.Period = (uint16_t)(period_ticks - 1);
  NIXIE_PWM_TIM_HANDLE.Init.RepetitionCounter = 0;
  NIXIE_PWM_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  NIXIE_PWM_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  __HAL_TIM_SET_COMPARE(&NIXIE_PWM_TIM_HANDLE, NIXIE_PWM_TIM_CHANNEL, NIXIE_PWM_TIM_HANDLE.Init.Period * NIXIE_PWM_DUTY_CYCLE / 100 - 1);
  if (HAL_TIM_PWM_Init(&NIXIE_PWM_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&NIXIE_PWM_TIM_HANDLE, NIXIE_PWM_TIM_CHANNEL) != HAL_OK)
  {
    Error_Handler();  
  }
}

static void nixie_stop_pwm_timer(void)
{
  if (HAL_TIM_PWM_Stop(&NIXIE_PWM_TIM_HANDLE, NIXIE_PWM_TIM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint64_t nixie_calculate_lptim_ticks_ms(const LPTIM_HandleTypeDef * const lptim, uint32_t time_period_ms)
{
  if (!lptim || !time_period_ms)
  {
    return 0;
  }
  // PCLK1 = APB1, frequency of LPTIM timers doesn't get doubled if APB prescaler != 1.
  // Time period in ms -> divide by 1000. 
  uint64_t rv = ((uint64_t) time_period_ms) * (HAL_RCC_GetPCLK1Freq() / 1000);
  switch (lptim->Init.Clock.Prescaler)
  {
    case LPTIM_PRESCALER_DIV1:
      break;
    case LPTIM_PRESCALER_DIV2:
      rv /= 2;
      break;
    case LPTIM_PRESCALER_DIV4:
      rv /= 4;
      break;
    case LPTIM_PRESCALER_DIV8:
      rv /= 8;
      break;
    case LPTIM_PRESCALER_DIV16:
      rv /= 16;
      break;
    case LPTIM_PRESCALER_DIV32:
      rv /= 32;
      break;
    case LPTIM_PRESCALER_DIV64:
      rv /= 64;
      break;
    case LPTIM_PRESCALER_DIV128:
      rv /= 128;
      break;
    default:
      Error_Handler();
      break;
  }
  return rv;
}

static uint32_t nixie_calculate_period_lptim_ms(const LPTIM_HandleTypeDef * const lptim, uint64_t period_ticks_ms)
{
  switch (lptim->Init.Clock.Prescaler)
  {
    case LPTIM_PRESCALER_DIV1:
      break;
    case LPTIM_PRESCALER_DIV2:
      period_ticks_ms *= 2;
      break;
    case LPTIM_PRESCALER_DIV4:
      period_ticks_ms *= 4;
      break;
    case LPTIM_PRESCALER_DIV8:
      period_ticks_ms *= 8;
      break;
    case LPTIM_PRESCALER_DIV16:
      period_ticks_ms *= 16;
      break;
    case LPTIM_PRESCALER_DIV32:
      period_ticks_ms *= 32;
      break;
    case LPTIM_PRESCALER_DIV64:
      period_ticks_ms *= 64;
      break;
    case LPTIM_PRESCALER_DIV128:
      period_ticks_ms *= 128;
      break;
    default:
      Error_Handler();
      break;
  }
  period_ticks_ms /= HAL_RCC_GetPCLK1Freq() / 1000;
  return period_ticks_ms;
}

static uint64_t nixie_calculate_tim_ticks_ms(const TIM_HandleTypeDef * const tim, uint32_t time_period_ms)
{
  if (!tim || !time_period_ms)
  {
    return 0;
  }
  uint64_t rv;
  // Some timers are on the APB2 bus, some on the APB1 one.
  // This list accounts for an STM32WB MCU. Update accordingly if a different MCU is used. 
  if (tim->Instance == TIM2)
  {
    rv = HAL_RCC_GetPCLK1Freq();
    // Timer frequency is double the bus speed if bus prescaler != 1.
    rv *= LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1? 1: 2;
  }
  // TIM1, TIM16, TIM17
  else
  {
    rv = HAL_RCC_GetPCLK2Freq();
    // Timer frequency is double the bus speed if bus prescaler != 1.
    rv *= LL_RCC_GetAPB2Prescaler() == LL_RCC_APB2_DIV_1? 1: 2;
  }
  rv *= time_period_ms;
  // Time period in ms -> divide by 1000. Divide in a separate command as time period is less likely to be % 1000.
  rv /= 1000;
  return rv;
}

static int nixie_compare_nixiealarmtime(NixieAlarmTime const * const a, NixieAlarmTime const * const b)
{
  if (b->hours > a->hours)
  {
    return -1;
  }
  if (a->hours > b->hours)
  {
    return 1;
  }
  if (b->minutes > a->minutes)
  {
    return -1;
  }
  if (a->minutes > b->minutes)
  {
    return 1;
  }
  return (a->seconds > b->seconds) - (b->seconds > a->seconds);
}

static void nixie_remove_alarm(NixieController * const nc, NixieAlarmTime const * const time)
{
  for (int i = 0; i < nc->active_alarms_size; ++i)
  {
    if (!nixie_compare_nixiealarmtime(time, &(nc->active_alarms[i].time)))
    {
      if (!(--(nc->active_alarms[i].cnt)))
      {
        memmove(&(nc->active_alarms[i]), &(nc->active_alarms[i + 1]), (nc->active_alarms_size - i + 1) * sizeof(nc->active_alarms[i]));
        --(nc->active_alarms_size);
      }
    }
  }
}

static void nixie_add_alarm(NixieController * const nc, NixieAlarmTime const * const time)
{
  int idx = 0;
  int cmp_rv;
  for (; idx < nc->active_alarms_size; ++idx)
  {
    if ((cmp_rv = nixie_compare_nixiealarmtime(time, &(nc->active_alarms[idx].time))) < 1)
    {
      break;
    }
  }

  // A new alarm time is being inserted to the end of the array
  if (idx == nc->active_alarms_size){
    // Comparison should never be true, there is probably an error in the code if it is.
    if (nc->active_alarms_size >= NIXIE_ALARMS_MAX)
    {
      return;
    }
    nc->active_alarms[idx].time = *time;
    nc->active_alarms[idx].cnt = 1;
    ++(nc->active_alarms_size);
  }  
  // A new alarm time is being inserted in the middle of the array.
  else if (cmp_rv)
  {
    // Comparison should never be true, there is probably an error in the code if it is.
    if (nc->active_alarms_size >= NIXIE_ALARMS_MAX)
    {
      return;
    }
    memmove(&(nc->active_alarms[idx + 1]), &(nc->active_alarms[idx]), (nc->active_alarms_size - idx + 1) * sizeof(nc->active_alarms[idx]));
    nc->active_alarms[idx].time = *time;
    nc->active_alarms[idx].cnt = 1;
    ++(nc->active_alarms_size);
  }
  // Array already contains said time.
  else
  {
    ++(nc->active_alarms[idx].cnt);
  }
  return;
}

static void nixie_update_lptim_timer(NixieController * const nc, NixieLptimUpdateType update_type)
{
  LPTIM_HandleTypeDef *lptim;
  uint64_t *ticks_total;
  uint64_t *ticks_left;
  pLPTIM_CallbackTypeDef no_overflow_callback;
  pLPTIM_CallbackTypeDef overflow_callback;
  if (!nc)
  {
    return;
  }
  switch (update_type)
  {
    case NIXIE_UPDATE_LED_BLINK_LPTIM:
      lptim = &NIXIE_LED_TIM_HANDLE;
      ticks_total = &(nc->led_blink_interval_ms_ticks);
      ticks_left = &(nc->led_blink_ticks_left);
      no_overflow_callback = nixie_led_timer_cb_no_overflow;
      overflow_callback = nixie_led_timer_cb_overflow;
      break;
    case NIXIE_UPDATE_DISPLAY_BLINK_LPTIM:
      lptim = &NIXIE_BLANK_TIM_HANDLE;
      ticks_total = &(nc->display_blink_interval_ms_ticks);
      ticks_left = &(nc->display_blink_ticks_left);
      no_overflow_callback = nixie_blank_timer_cb_no_overflow;
      overflow_callback = nixie_blank_timer_cb_overflow;
      break;
    case NIXIE_UPDATE_ALARM_RINGING_LPTIM:
      lptim = &NIXIE_RINGING_TIM_HANDLE;
      ticks_total = &(nc->alarm_ringing_duration_ms_ticks);
      ticks_left = &(nc->ringing_duration_ticks_left);
      no_overflow_callback = nixie_ringing_timer_alarm_cb_no_overflow;
      overflow_callback = nixie_ringing_timer_alarm_cb_overflow;
      break;
    case NIXIE_UPDATE_TIMER_RINGING_LPTIM:
      lptim = &NIXIE_RINGING_TIM_HANDLE;
      ticks_total = &(nc->timer_ringing_duration_ms_ticks);
      ticks_left = &(nc->ringing_duration_ticks_left);
      no_overflow_callback = nixie_ringing_timer_timer_cb_no_overflow;
      overflow_callback = nixie_ringing_timer_timer_cb_overflow;
      break;
    default:
      return;
      break;
  }
  if (HAL_LPTIM_Counter_Stop_IT(lptim) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LPTIM_CLEAR_FLAG(lptim, LPTIM_IT_ARRM);
  if (!(*ticks_total))
  {
    return;
  }
  if ((*ticks_total) > LPTIM_ARR_ARR)
  {
    if (HAL_LPTIM_RegisterCallback(lptim, HAL_LPTIM_AUTORELOAD_MATCH_CB_ID, overflow_callback) != HAL_OK)
    {
      Error_Handler();
    }
    (*ticks_left) = (*ticks_total) - LPTIM_ARR_ARR - 1;
    if (HAL_LPTIM_Counter_Start_IT(lptim, LPTIM_ARR_ARR) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {
    if (HAL_LPTIM_RegisterCallback(lptim, HAL_LPTIM_AUTORELOAD_MATCH_CB_ID, no_overflow_callback) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_LPTIM_Counter_Start_IT(lptim, (*ticks_total) - 1) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

static uint32_t nixie_calculate_dst_state(RTC_TimeTypeDef const * const time, RTC_DateTypeDef const * const date, NixieDstCalculationType type)
{
  uint32_t rv;
  // April-September range always should always be summer time.
  if (date->Month >= RTC_MONTH_APRIL && date->Month <= RTC_MONTH_SEPTEMBER)
  {
    rv = NIXIE_RTC_DST_STORE_SUMMER_TIME;
  }
  // November-February range always should always be standard time.
  else if (date->Month <= RTC_MONTH_FEBRUARY || date->Month >= RTC_MONTH_NOVEMBER)
  {
    rv = NIXIE_RTC_DST_STORE_STANDARD_TIME;
  }
  // March can be both, further checks needed.
  else if (date->Month == RTC_MONTH_MARCH)
  {
    // DST changes from standard to summer time on the last sunday in the month at 2:00.
    // DST change happens in the last 7 days of the week. The days outside of the range are always in standard time.
    if (date->Date <= NIXIE_DAYS_IN_MARCH - 7)
    {
      rv = NIXIE_RTC_DST_STORE_STANDARD_TIME;
    }
    else if (date->WeekDay == RTC_WEEKDAY_SUNDAY && time->Hours >= 2)
    {
      rv = NIXIE_RTC_DST_STORE_SUMMER_TIME; 
    }
    else if (date->WeekDay == RTC_WEEKDAY_SUNDAY)
    {
      rv = NIXIE_RTC_DST_STORE_STANDARD_TIME;
    }
    // DST change might happen while the device is out of power.
    // We can check for it by trying whether the next sunday date overflows the number of days in the month.
    else if ((date->Date + RTC_WEEKDAY_SUNDAY - date->WeekDay) > NIXIE_DAYS_IN_MARCH)
    {
      rv = NIXIE_RTC_DST_STORE_SUMMER_TIME; 
    }
    else
    {
      rv = NIXIE_RTC_DST_STORE_STANDARD_TIME;
    }
  }
  // October can be both, further checks needed.
  else
  {
    // DST changes from summer to standard time on the last sunday in the month at 3:00.
    // DST change happens in the last 7 days of the week. The days outside of the range are always in standard time.
    if (date->Date <= NIXIE_DAYS_IN_OCTOBER - 7)
    {
      rv = NIXIE_RTC_DST_STORE_SUMMER_TIME;
    }
    else if (date->WeekDay == RTC_WEEKDAY_SUNDAY && time->Hours >= 3)
    {
      rv = NIXIE_RTC_DST_STORE_STANDARD_TIME;
    }
    // If the current time is 2:xx and the store operation says the clock is in standard time, assume that the clock was adjusted correctly.
    // This logic should however only be used if we are checking for DST change. If we are calculating DST store bit value when setting a new RTC time, this assumption should not be done.
    else if (date->WeekDay == RTC_WEEKDAY_SUNDAY && time->Hours >= 2 && type == NIXIE_DST_CALCULATION_UPDATE
            && HAL_RTC_DST_ReadStoreOperation(&NIXIE_RTC_HANDLE) == NIXIE_RTC_DST_STORE_STANDARD_TIME)
    {
      rv = NIXIE_RTC_DST_STORE_STANDARD_TIME;
    }
    else if (date->WeekDay == RTC_WEEKDAY_SUNDAY)
    {
      rv = NIXIE_RTC_DST_STORE_SUMMER_TIME; 
    }
    // DST change might happen while the device is out of power.
    // We can check for it by trying whether the next sunday date overflows the number of days in the month.
    else if ((date->Date + RTC_WEEKDAY_SUNDAY - date->WeekDay) > NIXIE_DAYS_IN_OCTOBER)
    {
      rv = NIXIE_RTC_DST_STORE_STANDARD_TIME; 
    }
    else
    {
      rv = NIXIE_RTC_DST_STORE_SUMMER_TIME;
    }
  }
  return rv;
}

static NixieSetDigitArrayStatus nixie_set_digit_array_clock(NixieController const * const nc, uint8_t array[const NIXIE_OPT_NUM])
{
  uint16_t year = ((nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_YEAR_MSB_IDX] << CHAR_BIT) | nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_YEAR_LSB_IDX]) % 100;
  switch (nc->clock_state)
  {
    case NIXIE_CLOCK_STATE_TIME:
      array[NIXIE_HOURS_TENS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_HOURS_IDX] / 10;
      array[NIXIE_HOURS_UNITS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_HOURS_IDX] % 10;
      array[NIXIE_MINUTES_TENS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MINUTES_IDX] / 10;
      array[NIXIE_MINUTES_UNITS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MINUTES_IDX] % 10;
      array[NIXIE_SECONDS_TENS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_SECONDS_IDX] / 10;
      array[NIXIE_SECONDS_UNITS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_SECONDS_IDX] % 10;
      return NIXIE_DIGIT_ARRAY_NOT_EMPTY;
      break;
    case NIXIE_CLOCK_STATE_DATE:
      if (nc->date_format == NIXIE_DATE_FORMAT_EU)
      {
        array[NIXIE_EU_DATE_TENS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_DATE_IDX] / 10;
        array[NIXIE_EU_DATE_UNITS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_DATE_IDX] % 10;
        array[NIXIE_EU_MONTH_TENS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MONTH_IDX] / 10;
        array[NIXIE_EU_MONTH_UNITS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MONTH_IDX] % 10;
        array[NIXIE_EU_YEAR_TENS_OPT] = year / 10;
        array[NIXIE_EU_YEAR_UNITS_OPT] = year % 10;
      }
      // US Format.
      else
      {
        array[NIXIE_US_DATE_TENS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_DATE_IDX] / 10;
        array[NIXIE_US_DATE_UNITS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_DATE_IDX] % 10;
        array[NIXIE_US_MONTH_TENS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MONTH_IDX] / 10;
        array[NIXIE_US_MONTH_UNITS_OPT] = nc->gatt_clock_datetime_buffer[NIXIE_DATETIME_MONTH_IDX] % 10;
        array[NIXIE_US_YEAR_TENS_OPT] = year / 10;
        array[NIXIE_US_YEAR_UNITS_OPT] = year % 10;
      }     
      return NIXIE_DIGIT_ARRAY_NOT_EMPTY;
      break;
    case NIXIE_CLOCK_STATE_NONE:
      return NIXIE_DIGIT_ARRAY_EMPTY;
      break;
    default:
      // An unexpected empty display should indicate code issues well.
      return NIXIE_DIGIT_ARRAY_EMPTY;
      break;
  }
}

static void nixie_set_digit_array_seconds(uint8_t array[const NIXIE_OPT_NUM], uint32_t seconds)
{
  uint32_t hours;
  uint32_t minutes;

  hours = seconds / 3600;
  seconds -= hours * 3600;
  // Nixie tube display only has 2 decimal positions for hours.
  hours %= 100;
  minutes = seconds / 60;
  seconds -= minutes * 60;
  array[NIXIE_HOURS_TENS_OPT] = hours / 10;
  array[NIXIE_HOURS_UNITS_OPT] = hours % 10;
  array[NIXIE_MINUTES_TENS_OPT] = minutes / 10;
  array[NIXIE_MINUTES_UNITS_OPT] = minutes % 10;
  array[NIXIE_SECONDS_TENS_OPT] = seconds / 10;
  array[NIXIE_SECONDS_UNITS_OPT] = seconds % 10;
}

static uint64_t nixie_calculate_display_blink_interval_ms_ticks(uint32_t display_blink_interval_ms)
{
  return nixie_calculate_lptim_ticks_ms(&NIXIE_BLANK_TIM_HANDLE, display_blink_interval_ms);
}

static void nixie_detoxify(NixieController * const nc)
{
  nc->display_state = NIXIE_DISPLAY_STATE_DETOX;
  nc->detoxification_idx = NIXIE_DETOX_ROUNDS_NUM * NIXIE_DIGITS_NUM - 1;
  if (HAL_TIM_Base_Stop_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  NIXIE_DISPLAY_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  NIXIE_DISPLAY_TIM_HANDLE.Init.Prescaler = 0;
  NIXIE_DISPLAY_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
  NIXIE_DISPLAY_TIM_HANDLE.Init.RepetitionCounter = 0;
  NIXIE_DISPLAY_TIM_HANDLE.Init.Period = nixie_calculate_tim_ticks_ms(&NIXIE_DISPLAY_TIM_HANDLE, NIXIE_DETOX_DIGIT_DURATION_MS) - 1;
  NIXIE_DISPLAY_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  // Init function generates an update event. We do not want the interrupt to happen for this particular event.
  __HAL_TIM_CLEAR_IT(&NIXIE_DISPLAY_TIM_HANDLE, TIM_IT_UPDATE);
  if (HAL_TIM_RegisterCallback(&NIXIE_DISPLAY_TIM_HANDLE, HAL_TIM_PERIOD_ELAPSED_CB_ID, nixie_display_timer_detoxification_cb) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&NIXIE_DISPLAY_TIM_HANDLE) != HAL_OK)
  {
    Error_Handler();  
  }
  UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_UPDATE_DISPLAY_ID, CFG_SCH_PRIO_1);
}