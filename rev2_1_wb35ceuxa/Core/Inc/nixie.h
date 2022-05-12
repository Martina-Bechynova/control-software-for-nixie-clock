/** 
 * @file nixie.h
 * @brief Header file for the Nixie Clock application controller and any helper function and structures.
 */

#ifndef NIXIE_H
#define NIXIE_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32wbxx_hal.h"
#include "custom_stm.h"

/** GPIO INT1 pin port. */
#define NIXIE_INT1_PIN_PORT (GPIOB)
/** GPIO INT1 pin number. */
#define NIXIE_INT1_PIN_NUMBER (GPIO_PIN_5)
/** GPIO INT2 pin port. */
#define NIXIE_INT2_PIN_PORT (GPIOB)
/** GPIO INT2 pin number. */
#define NIXIE_INT2_PIN_NUMBER (GPIO_PIN_4)
/** GPIO BLANK pin port. */
#define NIXIE_BLANK_PIN_PORT (GPIOA)
/** GPIO BLANK pin number. */
#define NIXIE_BLANK_PIN_NUMBER (GPIO_PIN_6)
/** GPIO BLANK pin value to deactivate display blanking. */
#define NIXIE_BLANK_PIN_NOT_BLANK (GPIO_PIN_SET)
/** GPIO BLANK pin value to activate display blanking. */
#define NIXIE_BLANK_PIN_BLANK (GPIO_PIN_RESET)
/** GPIO LATCH pin port. */
#define NIXIE_LATCH_PIN_PORT (GPIOA)
/** GPIO LATCH pin number. */
#define NIXIE_LATCH_PIN_NUMBER (GPIO_PIN_4)
/** GPIO LATCH pin value to deactivate display register latch. */
#define NIXIE_LATCH_PIN_NOT_LATCHED (GPIO_PIN_SET)
/** GPIO LATCH pin value to activate display register latch. */
#define NIXIE_LATCH_PIN_LATCHED (GPIO_PIN_RESET)
/** GPIO LED1 pin port. */
#define NIXIE_LED1_PIN_PORT (GPIOB)
/** GPIO LED1 pin number. */
#define NIXIE_LED1_PIN_NUMBER (GPIO_PIN_0)
/** GPIO LED2 pin port. */
#define NIXIE_LED2_PIN_PORT (GPIOB)
/** GPIO LED2 pin number. */
#define NIXIE_LED2_PIN_NUMBER (GPIO_PIN_1)
/** GPIO LED3 pin port. */
#define NIXIE_LED3_PIN_PORT (GPIOA)
/** GPIO LED3 pin number. */
#define NIXIE_LED3_PIN_NUMBER (GPIO_PIN_12)
/** GPIO LED4 pin port. */
#define NIXIE_LED4_PIN_PORT (GPIOA)
/** GPIO LED4 pin number. */
#define NIXIE_LED4_PIN_NUMBER (GPIO_PIN_15)
/** GPIO LEDx pin value to turn LED on. */
#define NIXIE_LED_PIN_ON (GPIO_PIN_SET)
/** GPIO LEDx pin value to turn LED off. */
#define NIXIE_LED_PIN_OFF (GPIO_PIN_RESET)

/** HAL handle for the I2C module. */
#define NIXIE_I2C_HANDLE (hi2c1)
/** HAL handle for the SPI module used for display communication. */
#define NIXIE_DISPLAY_SPI_HANDLE (hspi1)
/** HAL handle for the timer used for PWM generation. */
#define NIXIE_PWM_TIM_HANDLE (htim1)
/** HAL handle for the timer used for handling display data. */
#define NIXIE_DISPLAY_TIM_HANDLE (htim2)
/** HAL handle for the timer used for LED blinking. */
#define NIXIE_LED_TIM_HANDLE (hlptim1)
/** HAL handle for the timer used for display blanking. */
#define NIXIE_BLANK_TIM_HANDLE (NIXIE_LED_TIM_HANDLE)
/** HAL handle for the timer used for alarm/timer ringing. */
#define NIXIE_RINGING_TIM_HANDLE (hlptim2)
/** HAL handle for the RTC module. */
#define NIXIE_RTC_HANDLE (hrtc)

/** Total number of configurable alarms. */
#define NIXIE_ALARMS_MAX (10)
/** Number of digit segments on a nixie tube. */
#define NIXIE_DIGITS_NUM (10)

/** Maximum digit value */
#define NIXIE_DIGITS_MAX_VALUE (9)
/** Number of display nixie segments. */
#define NIXIE_OPT_NUM (6)

/** GATT stopwatch status value indicating that the stopwatch is paused. */
#define NIXIE_STOPWATCH_PAUSED (0U)
/** GATT stopwatch status value indicating that the stopwatch is on. */
#define NIXIE_STOPWATCH_ON (1U)
/** GATT stopwatch status value indicating that the stopwatch is off. */
#define NIXIE_STOPWATCH_OFF (2U)

/** GATT timer status value indicating that the timer is paused. */
#define NIXIE_TIMER_PAUSED (0U)
/** GATT timer status value indicating that the timer is on. */
#define NIXIE_TIMER_ON (1U)
/** GATT timer status value indicating that the timer is off. */
#define NIXIE_TIMER_OFF (2U)

/** GATT alarm status value indicating that the alarm is off. */
#define NIXIE_ALARM_OFF (0U)
/** GATT alarm status value indicating that the alarm is on. */
#define NIXIE_ALARM_ON (1U)

/** GATT date format value indicating that the date should be shown in EU format. */
#define NIXIE_DATE_FORMAT_EU (0U)
/** GATT date format value indicating that the date should be shown in US format. */
#define NIXIE_DATE_FORMAT_US (1U)

/** Seconds index in the GATT Date Time SIG characteristic. */
#define NIXIE_DATETIME_SECONDS_IDX (6)
/** Minutes index in the GATT Date Time SIG characteristic. */
#define NIXIE_DATETIME_MINUTES_IDX (5)
/** Hours index in the GATT Date Time SIG characteristic. */
#define NIXIE_DATETIME_HOURS_IDX (4)
/** Date index in the GATT Date Time SIG characteristic. */
#define NIXIE_DATETIME_DATE_IDX (3)
/** Month index in the GATT Date Time SIG characteristic. */
#define NIXIE_DATETIME_MONTH_IDX (2)
/** Index of the most signifact byte of the year in the GATT Date Time SIG characteristic. */
#define NIXIE_DATETIME_YEAR_MSB_IDX (1)
/** Index of the least signifact byte of the year in the GATT Date Time SIG characteristic. */
#define NIXIE_DATETIME_YEAR_LSB_IDX (0)

/** Accelerometer I2C address */
#define NIXIE_ACCLRM_I2C_ADDR (0x53 << 1)
/** Accelerometer DEVID register address. */
#define NIXIE_ACCLRM_DEVID_ADDR (0x0)
/** Accelerometer DEVID register byte size. */
#define NIXIE_ACCLRM_DEVID_SIZE (1U)
/** Accelerometer THRESH_TAP register address. */
#define NIXIE_ACCLRM_THRESH_TAP_ADDR (0x1D)
/** Accelerometer THRESH_TAP register byte size. */
#define NIXIE_ACCLRM_THRESH_TAP_SIZE (1U)
/** Accelerometer THRESH_TAP register value after initialization. */
#define NIXIE_ACCLRM_THRESH_TAP_VALUE (0)
/** Accelerometer OFSX register address. */
#define NIXIE_ACCLRM_OFSX_ADDR (0x1E)
/** Accelerometer OFSX register byte size. */
#define NIXIE_ACCLRM_OFSX_SIZE (1U)
/** Accelerometer OFSX register value after initialization. */
#define NIXIE_ACCLRM_OFSX_VALUE (0)
/** Accelerometer OFSY register address. */
#define NIXIE_ACCLRM_OFSY_ADDR (0x1F)
/** Accelerometer OFSY register byte size. */
#define NIXIE_ACCLRM_OFSY_SIZE (1U)
/** Accelerometer OFSY register value after initialization. */
#define NIXIE_ACCLRM_OFSY_VALUE (0)
/** Accelerometer OFSZ register address. */
#define NIXIE_ACCLRM_OFSZ_ADDR (0x20)
/** Accelerometer OFSZ register byte size. */
#define NIXIE_ACCLRM_OFSZ_SIZE (1U)
/** Accelerometer OFSZ register value after initialization. */
#define NIXIE_ACCLRM_OFSZ_VALUE (0)
/** Accelerometer DUR register address. */
#define NIXIE_ACCLRM_DUR_ADDR (0x21)
/** Accelerometer DUR register byte size. */
#define NIXIE_ACCLRM_DUR_SIZE (1U)
/** Accelerometer DUR register value after initialization. */
#define NIXIE_ACCLRM_DUR_VALUE (0)
/** Accelerometer LATENT register address. */
#define NIXIE_ACCLRM_LATENT_ADDR (0x22)
/** Accelerometer LATENT register byte size. */
#define NIXIE_ACCLRM_LATENT_SIZE (1U)
/** Accelerometer LATENT register value after initialization. */
#define NIXIE_ACCLRM_LATENT_VALUE (0)
/** Accelerometer WINDOW register address. */
#define NIXIE_ACCLRM_WINDOW_ADDR (0x23)
/** Accelerometer WINDOW register byte size. */
#define NIXIE_ACCLRM_WINDOW_SIZE (1U)
/** Accelerometer WINDOW register value after initialization. */
#define NIXIE_ACCLRM_WINDOW_VALUE (0)
/** Accelerometer THRESH_ACT register address. */
#define NIXIE_ACCLRM_THRESH_ACT_ADDR (0x24)
/** Accelerometer THRESH_ACT register byte size. */
#define NIXIE_ACCLRM_THRESH_ACT_SIZE (1U)
/** Accelerometer THRESH_ACT register value after initialization. */
#define NIXIE_ACCLRM_THRESH_ACT_VALUE (30)
/** Accelerometer THRESH_INACT register address. */
#define NIXIE_ACCLRM_THRESH_INACT_ADDR (0x25)
/** Accelerometer THRESH_INACT register byte size. */
#define NIXIE_ACCLRM_THRESH_INACT_SIZE (1U)
/** Accelerometer THRESH_INACT register value after initialization. */
#define NIXIE_ACCLRM_THRESH_INACT_VALUE (NIXIE_ACCLRM_THRESH_ACT_VALUE)
/** Accelerometer TIME_INACT register address. */
#define NIXIE_ACCLRM_TIME_INACT_ADDR (0x26)
/** Accelerometer TIME_INACT register byte size. */
#define NIXIE_ACCLRM_TIME_INACT_SIZE (1U)
/** Accelerometer TIME_INACT register value after initialization.*/
#define NIXIE_ACCLRM_TIME_INACT_VALUE (0)
/** Accelerometer INACT_CTL register address. */
#define NIXIE_ACCLRM_ACT_INACT_CTL_ADDR (0x27)
/** Accelerometer INACT_CTL register byte size. */
#define NIXIE_ACCLRM_ACT_INACT_CTL_SIZE (1U)
/** Accelerometer INACT_CTL register value after initialization. */
#define NIXIE_ACCLRM_ACT_INACT_CTL_VALUE (0xFF)
/** Accelerometer THRESH_FF register address. */
#define NIXIE_ACCLRM_THRESH_FF_ADDR (0x28)
/** Accelerometer THRESH_FF register byte size. */
#define NIXIE_ACCLRM_THRESH_FF_SIZE (1U)
/** Accelerometer THRESH_FF register value after initialization. */
#define NIXIE_ACCLRM_THRESH_FF_VALUE (0)
/** Accelerometer TIME_FF register address. */
#define NIXIE_ACCLRM_TIME_FF_ADDR (0x29)
/** Accelerometer TIME_FF register byte size. */
#define NIXIE_ACCLRM_TIME_FF_SIZE (1U)
/** Accelerometer TIME_FF register value after initialization. */
#define NIXIE_ACCLRM_TIME_FF_VALUE (0)
/** Accelerometer TAP_AXES register address. */
#define NIXIE_ACCLRM_TAP_AXES_ADDR (0x2A)
/** Accelerometer TAP_AXES register byte size. */
#define NIXIE_ACCLRM_TAP_AXES_SIZE (1U)
/** Accelerometer TAP_AXES register value after initialization. */
#define NIXIE_ACCLRM_TAP_AXES_VALUE (0x08)
/** Accelerometer ACT_TAP_STATUS register address. */
#define NIXIE_ACCLRM_ACT_TAP_STATUS_ADDR (0x2B)
/** Accelerometer ACT_TAP_STATUS register byte size. */
#define NIXIE_ACCLRM_ACT_TAP_STATUS_SIZE (1U)
/** Accelerometer BW_RATE register address. */
#define NIXIE_ACCLRM_BW_RATE_ADDR (0x2C)
/** Accelerometer BW_RATE register byte size. */
#define NIXIE_ACCLRM_BW_RATE_SIZE (1U)
/** Accelerometer BW_RATE register value after initialization. */
#define NIXIE_ACCLRM_BW_RATE_VALUE (0x0A)
/** Accelerometer POWER_CTL register address. */
#define NIXIE_ACCLRM_POWER_CTL_ADDR (0x2D)
/** Accelerometer POWER_CTL register byte size. */
#define NIXIE_ACCLRM_POWER_CTL_SIZE (1U)
/** Accelerometer POWER_CTL register value after initialization. */
#define NIXIE_ACCLRM_POWER_CTL_VALUE (0x0B)
/** Accelerometer POWER_CTL register value used to begin initialization of the accelerometer. */
#define NIXIE_ACCLRM_POWER_CTL_INIT_MODE_VALUE (0x0)
/** Accelerometer INT_ENABLE register address. */
#define NIXIE_ACCLRM_INT_ENABLE_ADDR (0x2E)
/** Accelerometer INT_ENABLE register byte size. */
#define NIXIE_ACCLRM_INT_ENABLE_SIZE (1U)
/** Accelerometer INT_ENABLE register value after initialization. */
#define NIXIE_ACCLRM_INT_ENABLE_VALUE (0x10)
/** Accelerometer INT_MAP register address. */
#define NIXIE_ACCLRM_INT_MAP_ADDR (0x2F)
/** Accelerometer INT_MAP register byte size. */
#define NIXIE_ACCLRM_INT_MAP_SIZE (1U)
/** Accelerometer INT_MAP register value after initialization. */
#define NIXIE_ACCLRM_INT_MAP_VALUE (0xEF)
/** Accelerometer INT_SOURCE register address. */
#define NIXIE_ACCLRM_INT_SOURCE_ADDR (0x30)
/** Accelerometer INT_SOURCE register byte size. */
#define NIXIE_ACCLRM_INT_SOURCE_SIZE (1U)
/** Accelerometer DATA_FORMAT register address. */
#define NIXIE_ACCLRM_DATA_FORMAT_ADDR (0x31)
/** Accelerometer DATA_FORMAT register byte size. */
#define NIXIE_ACCLRM_DATA_FORMAT_SIZE (1U)
/** Accelerometer DATA_FORMAT register value after initialization. */
#define NIXIE_ACCLRM_DATA_FORMAT_VALUE (0x08)
/** Accelerometer DATAX register address. */
#define NIXIE_ACCLRM_DATAX_ADDR (0x32)
/** Accelerometer DATAX register byte size. */
#define NIXIE_ACCLRM_DATAX_SIZE (2U)
/** Accelerometer DATAY register address. */
#define NIXIE_ACCLRM_DATAY_ADDR (0x34)
/** Accelerometer DATAY register byte size. */
#define NIXIE_ACCLRM_DATAY_SIZE (2U)
/** Accelerometer DATAZ register address. */
#define NIXIE_ACCLRM_DATAZ_ADDR (0x36)
/** Accelerometer DATAZ register byte size. */
#define NIXIE_ACCLRM_DATAZ_SIZE (2U)
/** Accelerometer FIFO_CTL register address. */
#define NIXIE_ACCLRM_FIFO_CTL_ADDR (0x38)
/** Accelerometer FIFO_CTL register byte size. */
#define NIXIE_ACCLRM_FIFO_CTL_SIZE (1U)
/** Accelerometer FIFO_CTL register value after initialization. */
#define NIXIE_ACCLRM_FIFO_CTL_VALUE (0x20)
/** Accelerometer FIFO_STATUS register address. */
#define NIXIE_ACCLRM_FIFO_STATUS_ADDR (0x39)
/** Accelerometer FIFO_STATUS register byte size. */
#define NIXIE_ACCLRM_FIFO_STATUS_SIZE (1U)

/** EEPROM I2C address. */
#define NIXIE_EEPROM_I2C_ADDR (0x50 << 1)
/** EEPROM address of display output HV data. */
#define NIXIE_EEPROM_DISPLAY_OUTPUT_HV_ADDR (0x0)
/** EEPROM byte size of display output HV data. */
#define NIXIE_EEPROM_DISPLAY_OUTPUT_HV_SIZE (1U)
/** EEPROM address of year hundreds copy. */
#define NIXIE_EEPROM_YEAR_HUNDREDS_ADDR (0x1)
/** EEPROM byte size of year hundreds copy. */
#define NIXIE_EEPROM_YEAR_HUNDREDS_SIZE (2U)
/** EEPROM address of GATT LED blink interval characteristic copy. */
#define NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_ADDR (0x3)
/** EEPROM byte size of GATT LED blink interval characteristic copy. */
#define NIXIE_EEPROM_LED_BLINK_INTERVAL_MS_SIZE (CUSTOM_STM_UINT32_CHAR_SIZE)
/** EEPROM address of GATT time display duration characteristic copy. */
#define NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_ADDR (0x7)
/** EEPROM byte size of GATT time display duration characteristic copy. */
#define NIXIE_EEPROM_TIME_DISPLAY_DURATION_MS_SIZE (CUSTOM_STM_UINT32_CHAR_SIZE)
/** EEPROM address of GATT date display duration characteristic copy. */
#define NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_ADDR (0xB)
/** EEPROM byte size of GATT date display duration characteristic copy. */
#define NIXIE_EEPROM_DATE_DISPLAY_DURATION_MS_SIZE (CUSTOM_STM_UINT32_CHAR_SIZE)
/** EEPROM address of GATT alarm ringing duration characteristic copy. */
#define NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_ADDR (0xF)
/** EEPROM byte size of GATT alarm ringing duration characteristic copy. */
#define NIXIE_EEPROM_ALARM_RINGING_DURATION_MS_SIZE (CUSTOM_STM_UINT32_CHAR_SIZE)
/** EEPROM address of GATT timer ringing duration characteristic copy. */
#define NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_ADDR (0x13)
/** EEPROM byte size of GATT timer ringing duration characteristic copy. */
#define NIXIE_EEPROM_TIMER_RINGING_DURATION_MS_SIZE (CUSTOM_STM_UINT32_CHAR_SIZE)
/** EEPROM address of GATT date format characteristic copy. */
#define NIXIE_EEPROM_DATE_FORMAT_ADDR (0x17)
/** EEPROM byte size of GATT date format characteristic copy. */
#define NIXIE_EEPROM_DATE_FORMAT_SIZE (CUSTOM_STM_DATE_FORMAT_CHAR_SIZE)

/** Byte size of uint32_t. */
#define NIXIE_UINT32_SIZE (4U)
/** Byte size of uint16_t. */
#define NIXIE_UINT16_SIZE (2U)

/** Display states. */
typedef enum
{
  /** The display is in clock mode. */
  NIXIE_DISPLAY_STATE_CLOCK,
  /** The display is in timer mode. */
  NIXIE_DISPLAY_STATE_TIMER,
  /** The display is in stopwatch mode. */
  NIXIE_DISPLAY_STATE_STOPWATCH,
  /** The display is in tube detoxification mode. */
  NIXIE_DISPLAY_STATE_DETOX
} NixieDisplayState;

/** Clock states. */
typedef enum
{
  /** The clock is currently showing the time. */
  NIXIE_CLOCK_STATE_TIME,
  /** The clock is currently showing the date. */
  NIXIE_CLOCK_STATE_DATE,
  /** No data is being displayed in clock mode. */
  NIXIE_CLOCK_STATE_NONE
} NixieClockState;

/** LED states. */
typedef enum
{
  /** LEDs are turned off. */
  NIXIE_LED_STATE_OFF,
  /** LEDs are turned on. */
  NIXIE_LED_STATE_ON
} NixieLedState;

/** Ringing states. */
typedef enum
{
  /** Ringing is turned off. */
  NIXIE_RINGING_STATE_OFF,
  /** Ringing was turned on by an alarm. */
  NIXIE_RINGING_STATE_ALARM,
  /** Ringing was turned on by the timer. */
  NIXIE_RINGING_STATE_TIMER
} NixieRingingState;

/** Alarm numbers; used when changing data on a specific alarm. */
typedef enum
{
  /** Data change occurred on alarm 1. */
  NIXIE_ALARM1 = 0,
  /** Data change occurred on alarm 2. */
  NIXIE_ALARM2,
  /** Data change occurred on alarm 3. */
  NIXIE_ALARM3,
  /** Data change occurred on alarm 4. */
  NIXIE_ALARM4,
  /** Data change occurred on alarm 5. */
  NIXIE_ALARM5,
  /** Data change occurred on alarm 6. */
  NIXIE_ALARM6,
  /** Data change occurred on alarm 7. */
  NIXIE_ALARM7,
  /** Data change occurred on alarm 8. */
  NIXIE_ALARM8,
  /** Data change occurred on alarm 9. */
  NIXIE_ALARM9,
  /** Data change occurred on alarm 10. */
  NIXIE_ALARM10
} NixieAlarmNumber;

/** Enum indicating the ringing source. */
typedef enum
{
  /** The ringing was triggered by an alarm. */
  NIXIE_RINGING_SOURCE_ALARM,
  /** The ringing was triggered by the timer. */
  NIXIE_RINGING_SOURCE_TIMER
} NixieRingingSource;

/** Enum indicating the BLANK pin state. */
typedef enum
{
  /** Display isn't being blanked. */
  NIXIE_BLANK_STATE_OFF,
  /** Display is being blanked. */
  NIXIE_BLANK_STATE_ON
} NixieBlankState;

/** Alarm time structure. */
typedef struct
{
  /** Alarm seconds. */
  uint8_t seconds;
  /** Alarm minutes. */
  uint8_t minutes;
  /** Alarm hours. */
  uint8_t hours;
} NixieAlarmTime;


/** Alarm time structure with a duplicate counter. */
typedef struct
{
  /** Alarm time. */
  NixieAlarmTime time;
  /** The number of alarms set for the particular time. */
  uint8_t cnt;
} NixieAlarmCount;

/**
 * Alarm information structure.
 * This structure is used to record information about an alarm GATT service.
 */
typedef struct
{
  /** Set-up time on the specific alarm */
  NixieAlarmTime time;
  /** On/off status of the alarm */
  uint8_t status;
} NixieAlarmInfo;

/**
 * The application controller structure.
 * @note All date/time data expect binary format.
 */
typedef struct
{
  /** Display state. */
  NixieDisplayState display_state;
  /** Clock state. */
  NixieClockState clock_state;
  /** LED state. */
  NixieLedState led_state;
  /** BLANK pin state. */
  NixieBlankState blank_state;
  /** Ringing state. */
  NixieRingingState ringing_state;
  /** Buffer for keeping the current clock date time value. Used to synchronize the server and display time and to check for date/time updates. */
  uint8_t gatt_clock_datetime_buffer[CUSTOM_STM_DATETIME_CHAR_SIZE];
  /** An array containing all active alarms. */
  NixieAlarmCount active_alarms[NIXIE_ALARMS_MAX];
  /** Index of the scheduled active alarm. */
  uint8_t active_alarms_scheduled_idx;
  /** The number of valid records in the @ref active_alarms array. */
  uint8_t active_alarms_size;
  /** An array containg a copy of all GATT server alarms. */
  NixieAlarmInfo alarms_info[NIXIE_ALARMS_MAX];
  /** The year hundreds. */
  uint16_t year_hundreds;
  /** Value used to check RTC year overflow. */
  uint8_t prev_rtc_year; 
  /** Current stopwatch time. */
  uint32_t stopwatch_time_s;
  /** A copy of the MCU timer CNT register value; used when starting the stopwatch after pausing it to restore the subsecond count. */
  uint32_t stopwatch_tim_cnt;
  /** Stopwatch status. */
  uint8_t stopwatch_status;
  /** Current timer time. */
  uint32_t timer_time_s;
  /** A copy of the MCU timer CNT register value; used when starting the timer after pausing it to restore the subsecond count. */
  uint32_t timer_tim_cnt;
  /** Timer status. */
  uint8_t timer_status;

  /** LED blink interval in MCU timer ticks. */
  uint64_t led_blink_interval_ms_ticks;
  /** Time display duration. */
  uint32_t time_display_duration_ms;
  /** Date display duration. */
  uint32_t date_display_duration_ms;
  /** Alarm ringing duration in MCU timer ticks. */
  uint64_t alarm_ringing_duration_ms_ticks;
  /** Timer ringing duration in MCU timer ticks. */
  uint64_t timer_ringing_duration_ms_ticks;
  /** Date format. */
  uint8_t date_format;
  /** Display blink interval in ticks; only used if LED blinking is turned off. */
  uint64_t display_blink_interval_ms_ticks;
  /** MCU Timer ticks left until the desired led blinking interval period elapses. */
  uint64_t led_blink_ticks_left;
  /** MCU Timer ticks left until the desired ringing duration period elapses. */
  uint64_t ringing_duration_ticks_left;
  /** MCU Timer ticks left until the desired display blink period elapses. */
  uint64_t display_blink_ticks_left;
  /** Detoxification index; used to calculate which digit to detoxify. */
  uint8_t detoxification_idx;
  /** Display converter type used on the device. */
  uint8_t display_output_hv;
} NixieController;

/**
 * @brief Split uint32_t data into a byte array.
 * @param[out] array Array to write byte data to
 * @param[in] value Value to be saved into the array in bytes
 * @note Endianness determined by @ref NIXIE_GATT_SERVER_ENDIANNESS
 */
void nixie_uint32_to_uint8_array(uint8_t array[const NIXIE_UINT32_SIZE], uint32_t value);

/**
 * @brief Merge array byte data into an uint32_t variable.
 * @param[in] array Byte array
 * @return Value contained in byte array 
 * @note Endianness determined by @ref NIXIE_GATT_SERVER_ENDIANNESS
 */
uint32_t nixie_uint8_array_to_uint32(uint8_t const array[const NIXIE_UINT32_SIZE]);

/**
 * @brief Split uint16_t data into a byte array.
 * @param[out] array Array to write byte data to
 * @param[in] value Value to be saved into the array in bytes
 * @note Endianness determined by @ref NIXIE_GATT_SERVER_ENDIANNESS
 */
void nixie_uint16_to_uint8_array(uint8_t array[const NIXIE_UINT16_SIZE], uint16_t value);

/**
 * @brief Merge array byte data into an uint16_t variable.
 * @param[in] array Byte array
 * @return Value contained in byte array 
 * @note Endianness determined by @ref NIXIE_GATT_SERVER_ENDIANNESS
 */
uint16_t nixie_uint8_array_to_uint16(uint8_t const array[const NIXIE_UINT16_SIZE]);

/**
 * @brief Register HAL callbacks for peripherals that do not have a dedicated initialization function.
 */
void nixie_register_hal_callbacks(void);

/**
 * @brief Calculate the LED timer ticks needed to achieve the desired LED blink interval.
 * @param[in] led_blink_interval_ms The desired LED blink interval
 * @return LED timer ticks equal to the supplied interval
 */
uint64_t nixie_calculate_led_blink_interval_ms_ticks(uint32_t led_blink_interval_ms);

/**
 * @brief Calculate the ringing timer ticks needed to achieve the desired ringing duration.
 * @param[in] ringing_duration_ms The desired ringing duration
 * @return Ringing timer ticks equal to the supplied duration
 */
uint64_t nixie_calculate_ringing_duration_ms_ticks(uint32_t ringing_duration_ms);

/**
 * @brief Initialize GPIO pins.
 */
void nixie_init_gpio(void);

/**
 * @brief Enable battery charging.
 */
void nixie_enable_battery_charging(void);

/**
 * @brief Initialize RTC alarms.
 */
void nixie_init_rtc_alarms(void);

/**
 * @brief Transmit data to display SPI.
 * @param[in] data Data to be sent via display SPI
 */
void nixie_transmit_display_spi_data(uint64_t data);

/**
 * @brief Check whether the new date format value is valid.
 * @param[in] payload New date format value
 * @return Validity status
 */
bool nixie_is_valid_date_format(uint8_t payload);

/**
 * @brief Check whether the new alarm time value is valid.
 * @param[in] payload New alarm time value
 * @return Validity status
 */
bool nixie_is_valid_alarm_time(uint8_t const payload[const CUSTOM_STM_DATETIME_CHAR_SIZE]);

/**
 * @brief Check whether the new alarm status value is valid.
 * @param[in] alarm_status New alarm status value
 * @return Validity status
 */
bool nixie_is_valid_alarm_status(uint8_t alarm_status);

/**
 * @brief Check whether the new CALP value is valid.
 * @param[in] calp New CALP value
 * @return Validity status
 */
bool nixie_is_valid_rtc_calibration_calp(uint8_t calp);

/**
 * @brief Check whether the new date time value is valid.
 * @param[in] payload New date time value
 * @return Validity status
 */
bool nixie_is_valid_date_time(uint8_t const payload[const CUSTOM_STM_DATETIME_CHAR_SIZE]);

/**
 * @brief Update the date time characteristic in the GATT clock service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_clock_datetime(NixieController const * const nc);

/**
 * @brief Update the time characteristic in one of the GATT alarm services to match application data.
 * @param[in] nc Application controller
 * @param[in] alarm_number The number of the alarm that should be updated.
 */
void nixie_update_gatt_alarm_status(NixieController const * const nc, NixieAlarmNumber alarm_number);

/**
 * @brief Update the status characteristic in one of the GATT alarm services to match application data.
 * @param[in] nc Application controller
 * @param[in] alarm_number The number of the alarm that should be updated.
 */
void nixie_update_gatt_alarm_time(NixieController const * const nc, NixieAlarmNumber alarm_number);

/**
 * @brief Update the time characteristic in the GATT stopwatch service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_stopwatch_time_s(NixieController const * const nc);

/**
 * @brief Update the status characteristic in the GATT stopwatch service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_stopwatch_status(NixieController const * const nc);

/**
 * @brief Update the time characteristic in the GATT timer service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_timer_time_s(NixieController const * const nc);

/**
 * @brief Update the status characteristic in the GATT clock service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_timer_status(NixieController const * const nc);

/**
 * @brief Update the LED blink interval characteristic in the GATT settings service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_settings_led_blink_interval_ms(NixieController const * const nc);

/**
 * @brief Update the time display duration characteristic in the GATT settings service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_settings_time_display_duration_ms(NixieController const * const nc);

/**
 * @brief Update the date display duration characteristic in the GATT settings service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_settings_date_display_duration_ms(NixieController const * const nc);

/**
 * @brief Update the alarm ringing duration characteristic in the GATT settings service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_settings_alarm_ringing_duration_ms(NixieController const * const nc);

/**
 * @brief Update the timer ringing duration characteristic in the GATT settings service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_settings_timer_ringing_duration_ms(NixieController const * const nc);

/**
 * @brief Update the date format characteristic in the GATT settings service to match application data.
 * @param[in] nc Application controller
 */
void nixie_update_gatt_settings_date_format(NixieController const * const nc);

/**
 * @brief Update the CALP characteristic in the GATT RTC calibration service to match application data.
 */
void nixie_update_gatt_rtc_calibration_calp(void);

/**
 * @brief Update the CALM characteristic in the GATT RTC calibration service to match application data.
 */
void nixie_update_gatt_rtc_calibration_calm(void);

/**
 * @brief Calculate the day of the week from the date.
 * @param[in] year Year
 * @param[in] month Month
 * @param[in] date Date
 * @return Day of the week, a value of @ref RTC_WeekDay_Definitions
 */
uint8_t nixie_calculate_day_of_week(unsigned int year, unsigned int month, unsigned int date);

/**
 * @brief Initialize the data in a NixieController instance.
 * @param nc Application controller
 */
void nixie_init_controller(NixieController * const nc);

/**
 * @brief Set all controller variables according to the data in the parameters.
 * @param[in,out] nc Application controller
 * @param[in] time RTC time
 * @param[in] date RTC date
 * @param[in] year The year, including the century
 */
void nixie_set_datetime_members(NixieController * const nc, RTC_TimeTypeDef const * const time, RTC_DateTypeDef const * const date, uint16_t year);

/**
 * @brief Calculate the display data according to display state.
 * @param nc Application controller
 * @return Display data
 */
uint64_t nixie_calculate_display_data(NixieController * const nc);

/**
 * @brief Set the GATT clock date time characteristic copy according to RTC data.
 * @param[in] nc Application controller
 * @param[in] time RTC time
 * @param[in] date RTC date
 */
void nixie_set_gatt_clock_datetime_buffer(NixieController * const nc, RTC_TimeTypeDef const * const time, RTC_DateTypeDef const * const date);

/**
 * @brief Update RTC year overflow structure members
 * @param nc Application controller
 * @param current_year Current RTC year
 */
void nixie_update_rtc_year_overflow(NixieController * const nc, uint8_t current_year);

/**
 * @brief Check if controller datetime data needs to be updated.
 * @param[in] nc Application controller
 * @param[in] time RTC time
 * @param[in] date RTC date
 * @return Flag indicating whether the controller data needs to be updated.
 */
bool nixie_is_datetime_update_needed(NixieController const * const nc, RTC_TimeTypeDef const * const time, RTC_DateTypeDef const * const date);

/**
 * @brief Set the alarm time on one of the alarms.
 * @param[in,out] nc Application controller
 * @param[in] alarm_number Alarm number
 * @param[in] time New time value
 */
void nixie_set_alarm_time(NixieController * const nc, NixieAlarmNumber alarm_number, NixieAlarmTime const * const time);

/**
 * @brief Set the alarm status on one of the alarms.
 * @param[in,out] nc Application controller
 * @param[in] alarm_number Alarm number
 * @param[in] status New status value
 */
void nixie_set_alarm_status(NixieController * const nc, NixieAlarmNumber alarm_number, uint8_t status);

/**
 * @brief Initialize the accelerometer.
 * @return Initialization success flag
 */
bool nixie_init_accelerometer(void);

/**
 * @brief Write data to the accelerometer.
 * @param[in] data_addr Address to begin writing from
 * @param[out] data Data to write to the accelerometer
 * @param[in] data_size Number of bytes to write
 * @return Write operation success flag
 */
bool nixie_write_accelerometer(uint16_t data_addr, uint8_t * const data, uint16_t data_size);

/**
 * @brief Read accelerometer data.
 * @param[in] data_addr Address to begin reading from
 * @param[out] data Array to save read data into
 * @param[in] data_size Number of bytes to read
 * @return Read operation success flag
 */
bool nixie_read_accelerometer(uint16_t data_addr, uint8_t * const data, uint16_t data_size);

/**
 * @brief Initialize EEPROM data.
 * @return Initialization success flag
 */
bool nixie_init_eeprom(void);

/**
 * @brief Write data to EEPROM.
 * @param[in] data_addr Address to begin writing from
 * @param[out] data Data to write to EEPROM
 * @param[in] data_size Number of bytes to write
 * @return Write operation success flag
 */
bool nixie_write_eeprom(uint16_t data_addr, uint8_t * const data, uint16_t data_size);

/**
 * @brief Read EEPROM data.
 * @param[in] data_addr Address to begin reading from
 * @param[out] data Array to save read data into
 * @param[in] data_size Number of bytes to read
 * @return Read operation success flag
 */
bool nixie_read_eeprom(uint16_t data_addr, uint8_t * const data, uint16_t data_size);

/**
 * @brief Update DST state.
 * @details The functions determines whether a DST change is needed and updates the RTC module accordingly.
 */
void nixie_update_dst(void);

/**
 * @brief Initialize the GATT server data according to controller data.
 * @param nc Application controller
 */
void nixie_init_gatt_server(NixieController const * const nc);

/**
 * @brief Update the display blank pin.
 * @param[in] nc Application controller
 */
void nixie_update_blank_pin(NixieController const * const nc);

/**
 * @brief Update the LED timer according to controller data.
 * @details The function updates LED timer to match LED blink interval.
 * @param[in,out] nc Application controller
 */
void nixie_update_led_timer(NixieController * const nc);

/**
 * @brief Update the LEDS according to controller data.
 * @details The function first evaluates the state the LEDs should be in and updates the GPIO pins appropriately.
 * @param nc Application controller
 */
void nixie_update_leds(NixieController * const nc);

/**
 * @brief Update the clock timer according to controller data.
 * @details The function updates the display timer for clock mode. If both time and date should be shown, the update will reset the clock state to @ref NIXIE_CLOCK_STATE_TIME.
 * @param[in,out] nc Application controller
 */
void nixie_update_clock_timer(NixieController * const nc);

/**
 * @brief Reschedule the alarm.
 * @details The function reschedules the alarm. If no data change has occurred, it can simply move to the next alarm in line.
 *          If the alarm data has changed, it will check all alarms to find the closest one.
 * @param[in,out] nc Application controller
 * @param[in] alarms_changed Flag indicating whether alarm data has changed since last reschedule.
 */
void nixie_reschedule_alarm(NixieController * const nc, bool alarms_changed);

/**
 * @brief Start ringing.
 * @details The function turns on ringing for a duration of time determined by ringing source.
 * @param[in,out] nc Application controller
 * @param[in] source Ringing source
 */
void nixie_start_ringing(NixieController * const nc, NixieRingingSource source);

/**
 * @brief Stop ringing.
 * @details The function turns off ringing and handles any additional changes needed.
 * @param[in,out] nc Application controller
 */
void nixie_stop_ringing(NixieController * const nc);

/**
 * @brief Set timer time data in controller.
 * @param[in,out] nc Application controller
 * @param[in] time_s New timer time value
 */
void nixie_set_timer_time_s(NixieController * const nc, uint32_t time_s);

/**
 * @brief Start stopwatch mode.
 * @param[in,out] nc Application controller
 */
void nixie_start_stopwatch_mode(NixieController * const nc);

/**
 * @brief Stop stopwatch mode.
 * @details The function stops stopwatch mode and starts clock mode.
 * @param[in,out] nc Application controller
 */
void nixie_stop_stopwatch_mode(NixieController * const nc);

/**
 * @brief Pause stopwatch mode
 * @param[in,out] nc Application controller
 */
void nixie_pause_stopwatch_mode(NixieController * const nc);

/**
 * @brief Start timer mode.
 * @param[in,out] nc Application controller
 */
void nixie_start_timer_mode(NixieController * const nc);

/**
 * @brief Stop timer mode.
 * @details The function stops timer mode and starts clock mode.
 * @param[in,out] nc Application controller
 */
void nixie_stop_timer_mode(NixieController * const nc);

/**
 * @brief Pause timer mode.
 * @param[in,out] nc Application controller
 */
void nixie_pause_timer_mode(NixieController * const nc);

/**
 * @brief Process accelerometer activity event.
 * @param[in,out] nc Application controller
 */
void nixie_process_accelerometer_activity(NixieController * const nc);

/**
 * @brief Process hourly check alarm event.
 * @param[in,out] nc Application controller
 */
void nixie_process_hourly_check(NixieController * const nc);

#endif //NIXIE_H
