#ifndef _GP2Y0E03_H_
#define _GP2Y0E03_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>    
 

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define GP2Y0E          0x40//default
#define SHIFT_BYTE      0x02 //64 cm shift = 2 128 cm shift = 1
#define SHIFT_ADDR      0x35
#define DISTANCE_ADDR1  0x5E
#define DISTANCE_ADDR2  0x5F
#define RIGHT_EDGE_ADDR 0xF8 // C
#define LEFT_EDGE_ADDR  0xF9 // A
#define PEAK_EDGE_ADDR  0xFA // B     

#define E_FUSE_ADDR  0xC8 
#define VDD_PIN 22
#define DEBUG_I2C 0

void DS_init(char sladress);    
float DS_get_data(char sladress);    
void Ds_change( uint8_t new_address);
void DS_range(char adress,char distance);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif // _RC522_H