
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"


#define GPIO_PWM0A_OUT 13   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 12   //Set GPIO 16 as PWM0B

 void mcpwm_example_gpio_initialize(void);


/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
 void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
 void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);

/**
 * @brief motor stop
 */
 void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num);

