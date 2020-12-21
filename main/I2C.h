#ifndef _I2C_H_
#define _I2C_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>    
#include "driver/i2c.h"


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */


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

 //esp_err_t i2c_master_read_slave(i2c_port_t i2c_num,uint8_t sensor_addr, uint8_t *data_rd, size_t size);
 //esp_err_t i2c_master_write_slave(i2c_port_t i2c_num,uint8_t sensor_addr, uint8_t data_wr, size_t size);
 esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l);
 esp_err_t i2c_esp32_read(i2c_port_t i2c_num,uint8_t sensor_addr,uint8_t reg_addr, uint8_t * reag_data,uint8_t size);
 esp_err_t i2c_esp32_write(i2c_port_t i2c_num,uint8_t sensor_addr,uint8_t reg_addr, uint8_t  reag_data,uint8_t size);
 esp_err_t i2c_master_init(void);

  
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif // _RC522_H