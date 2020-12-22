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
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "GP2Y0E03.h"
#include "I2C.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;


/**
 * @brief i2c master initialization
 *
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}*/



/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

static void i2c_test_task(void *arg)
{
    int i = 0;
    esp_err_t ret=ESP_OK;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t sensor_data_h, sensor_data_l;
    int cnt = 0;
        uint8_t adrees = 0;
// double v;

    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", adrees, cnt++);
         gpio_set_level(4, 1);//0
         gpio_set_level(23, 1);//0
        
            //Ds_change(0x01);


      //ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
          // Ds_change(0x01);
        ret = i2c_esp32_read(I2C_MASTER_NUM, 0x08,0xC8,&sensor_data_h,1);
        
        /*if(cnt>127){
            cnt=0;
        }*/

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

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    DS_init(0x00);

    DS_range(0x08,0x01);

   // Ds_change(0x01);

    //ponteiro de função,label(reconhecimento no debug) ,espaço de memoria,ponteiro de precarregamento,prioridade,rando da task (id)
    //prioridade menori indice menor prioridade , maior indicie maior prioridade

    //xTaskCreate core randomico
    
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);


     //ponteiro de função,label(reconhecimento no debug) ,espaço de memoria,ponteiro de precarregamento,prioridade,rando da task (id),core
    
    //xTaskCreatePinnedToCore core especifico
    //xTaskCreatePinnedToCore(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL,0);

    //xTaskCreate core randomico
    //xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
}
