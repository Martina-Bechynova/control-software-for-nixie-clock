/** 
 * @file nixie_hal_weak_fn.c
 * @brief Source file containing any overriden weak functions from HAL API.
 */

#include "nixie.h"
#include "stm32_seq.h"
#include "app_conf.h"

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  if (gpio_pin == NIXIE_INT1_PIN_NUMBER)
  {
    UTIL_SEQ_SetTask(1 << CFG_TASK_NIXIE_PROCESS_ACCLRM_ACT_ID, CFG_SCH_PRIO_0);
  }
}