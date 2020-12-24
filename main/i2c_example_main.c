/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"

//#include "driver/i2c.h"
#include "sdkconfig.h"

#include "GP2Y0E03.h"
#include "I2C.h"

#include "PWM.h"

#include "GPIO.h"
static xQueueHandle gpio_evt_queue = NULL;

static const char *TAG = "i2c-example";

#define NACK_VAL 0x1                            /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;


static void i2c_test_task(void *arg)
{
    
    esp_err_t ret=ESP_OK;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
    
    uint8_t adrees = 0;
    uint8_t sensor_data_h, sensor_data_l;
    int cnt = 0;
   
// double v;

    while (1) {

        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", adrees, cnt++);

         gpio_set_level(4, 1);//0
         gpio_set_level(23, 1);//0
        
        ret = i2c_esp32_read(I2C_MASTER_NUM, 0x08,0xC8,&sensor_data_h,1);
        
        xSemaphoreTake(print_mux, portMAX_DELAY);

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
            
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ SENSOR( IR 0x10 )\n", task_idx);
            printf("*******************\n");
            printf("data_h: %02x\n", sensor_data_h);

        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
       
      /* v= */DS_get_data(0x08);
/*
            printf("*****I2C distancia******\n");
            printf("sensor_addr: %g\n", v);
            printf("*******************\n");*/

        xSemaphoreGive(print_mux);
        vTaskDelay((1000 * (task_idx + 1)) / portTICK_RATE_MS);
        //---------------------------------------------------
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        printf("brushed_motor_forward\n");
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        printf("brushed_motor_backward\n");
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        printf("brushed_motor_stop\n");    
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
 void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void gpio_task_example(void* arg)
{
    uint32_t io_num;
        printf("gpio_task_example...\n");

    while (1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());

    DS_init(0x00);

    DS_range(0x08,0x01);

    // Ds_change(0x01);
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 5, NULL);
    
   // gpio_init_driver(gpio_isr_t &gpio_isr_handler);
       gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_IO_0, 1);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

   //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);



    //ponteiro de função,label(reconhecimento no debug) ,espaço de memoria,ponteiro de precarregamento,prioridade,rando da task (id)
    //prioridade menori indice menor prioridade , maior indicie maior prioridade

    //xTaskCreate core randomico


    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 4, NULL);


     //ponteiro de função,label(reconhecimento no debug) ,espaço de memoria,ponteiro de precarregamento,prioridade,rando da task (id),core
    
    //xTaskCreatePinnedToCore core especifico
    //xTaskCreatePinnedToCore(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL,0);

    //xTaskCreate core randomico
    //xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
}
