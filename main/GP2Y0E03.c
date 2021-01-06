//https://github.com/LASER-UD/gp2y0e03_Psoc_5/blob/master/Workspace01/SensorDistancia.cydsn/GP2Y0E03.c
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "GP2Y0E03.h"
#include "I2C.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

void DS_init(char sladress)
{
    char buffer[12] = {};
    //Funcion de configuracion o de escritura de registro
    //I2C_Start();
     gpio_reset_pin(VDD_PIN);
    /* Set the GPIO as a push/pull output */
     gpio_set_direction(VDD_PIN, GPIO_MODE_OUTPUT);
     gpio_set_level(VDD_PIN, 1);//0

     gpio_reset_pin(23);
    /* Set the GPIO as a push/pull output */
     gpio_set_direction(23, GPIO_MODE_OUTPUT);
     gpio_set_level(23, 1);//0


     gpio_reset_pin(4);
    /* Set the GPIO as a push/pull output */
     gpio_set_direction(4, GPIO_MODE_OUTPUT);
     gpio_set_level(4, 1);//0
   // VDAC8_Start();
   // VDAC8_SetValue(0);
    /*     DS_beginwrite(sladress);
    I2C_MasterWriteByte(SHIFT_ADDR);//Pone direccion de memoria que quiere leer 
    I2C_MasterSendStop(); 
    DS_beginread(sladress);
    sprintf(buffer,"shift %02X\n\r",I2C_MasterReadByte(I2C_NAK_DATA));//lo codifica en ascci
    I2C_MasterSendStop();
    UART_PutString(buffer); */
}

char DS_read(uint8_t sensor_addr, uint8_t reg_addr)
{
    uint8_t data = 0;
    int32_t ret;

  ret = i2c_esp32_read(I2C_MASTER_NUM , sensor_addr, reg_addr, &data,1);

#if DEBUG_I2C
         if (ret == ESP_ERR_TIMEOUT) {
            printf("I2C Timeout");
        } else if (ret == ESP_OK) {

            printf("*******I2C read********\n");
            printf("sensor_addr: %02x reg_addr: %02x  data: %02x\n", sensor_addr,reg_addr,data);
            printf("*******************\n");
        } else {
            printf( " No ack, sensor not connected...skip...\n");
        }
#endif DEBUG_I2C
    return data;
} //adress es el slave adress del dispositivo, leer es la direccion del registro que se desea ver

void DS_write(uint8_t sensor_addr, uint8_t reg_addr, uint8_t  reag_data)
{
     int32_t ret;
  
ret = i2c_esp32_write(I2C_MASTER_NUM, sensor_addr, reg_addr, reag_data, 1);

//#if DEBUG_I2C
         if (ret == ESP_ERR_TIMEOUT) {
            printf( "I2C Timeout\n");
        } else if (ret == ESP_OK) {

            printf("*****I2C write******\n");
            printf("sensor_addr: %02x reg_addr: %02x\n", sensor_addr,reg_addr);
            printf("*******************\n");
        } else {
            printf( " No ack, sensor not connected...skip...\n");
        }
 //#endif DEBUG_I2C
}

void e_fuse_stage1()
{
    DS_write(GP2Y0E, 0xEC, 0xFF);
    //VDAC8_SetValue(206);
    gpio_set_level(VDD_PIN, 1);

}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 2
 * Data=0x00 is set in Address=0xC8.
 */
void e_fuse_stage2()
{
    DS_write(GP2Y0E, 0xC8, 0x00);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 3
 * Data=0x45 is set in Address=0xC9.
 * + programming bit #: 5 => 5 - 1 = 4
 * + bank value: 5 => Bank E
 */
void e_fuse_stage3()
{
    DS_write(GP2Y0E, 0xC9, 0x45);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 4
 * Data=0x01 is set in Address=0xCD.
 * (Data=0x01 for slave address being changed to 0x10(write) and 0x11(read))
 * @param new_address 0-15 (Default address is 8, 0x80 for writing and 0x81 for reading)
 */
void e_fuse_stage4(uint8_t new_address)
{

    DS_write(GP2Y0E, 0xCD, new_address);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 5
 * Data=0x01 is set in Address=0xCA.
 * Wait for 500us.
 */
void e_fuse_stage5()
{

    DS_write(GP2Y0E, 0xCA, 0x01);
    //CyDelay(500);
    vTaskDelay(0.05 / portTICK_PERIOD_MS);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 6
 * Data=0x00 is set in Address=0xCA.
 * Vpp terminal is grounded.
 */
void e_fuse_stage6()
{
    DS_write(GP2Y0E, 0xCA, 0x00);
    //VDAC8_SetValue(0);
    gpio_set_level(VDD_PIN, 0);

}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 7
 * Data=0x00 is set in Address=0xEF.
 * Data=0x40 is set in Address=0xC8.
 * Data=0x00 is set in Address=0xC8.
 */
void e_fuse_stage7()
{

    DS_write(GP2Y0E, 0xEF, 0x00);
    DS_write(GP2Y0E, 0xC8, 0x40);
    DS_write(GP2Y0E, 0xC8, 0x00);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 8
 * Data=0x06 is set in Address=0xEE.
 */
void e_fuse_stage8()
{

    DS_write(GP2Y0E, 0xEE, 0x06);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 9
 * Data=0xFF is set in Address=0xEC.
 * Data=0x03 is set in Address=0xEF.
 * Read out the data in Address=0x27.
 * Data=0x00 is set in Address=0xEF.
 * Data=0x7F is set in Address=0xEC.
 *
 * @return 0 for success, 1 for failure : 0x27[4:0] & 0b10000(0x10)
 */
uint8_t e_fuse_stage9()
{

    // Table.20 List of E-Fuse program flow and setting value
    DS_write(GP2Y0E, 0xEF, 0x00); // add this though it's missing in 12-6 Example of E-Fuse Programming
    DS_write(GP2Y0E, 0xEC, 0xFF);
    DS_write(GP2Y0E, 0xEF, 0x03);
    uint8_t check_value = DS_read(GP2Y0E, 0x27);
    uint8_t check = check_value & 0x1f;
    // When lower 5bits data[4:0] is 00001, E-Fuse program is finished.
    // When lower 5bits data[4:0] is not 00001, go to stage10(bit replacement).
    DS_write(GP2Y0E, 0xEF, 0x00);
    DS_write(GP2Y0E, 0xEC, 0x7F);
    if (check == 0b00000001)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void e_fuse_stage10_1_1()
{
    DS_write(GP2Y0E, 0xEC, 0xFF);
    //VDAC8_SetValue(206);
    //_vpp_on();
    gpio_set_level(VDD_PIN, 1);

}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-2
 * Data=0x37 is set in Address=0xC8.
 */
void e_fuse_stage10_2_1()
{

    DS_write(GP2Y0E, 0xC8, 0x37);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-3
 * Data=0x74 is set in Address=0xC9.
 */
void e_fuse_stage10_3_1()
{

    DS_write(GP2Y0E, 0xC9, 0x74);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-4
 * Data=0x04 is set in Address=0xCD.
 */
void e_fuse_stage10_4_1()
{

    DS_write(GP2Y0E, 0xCD, 0x04);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-5
 * Data=0x01 is set in Address=0xCA.
 * Wait for 500us.
 */
void e_fuse_stage10_5_1()
{

    DS_write(GP2Y0E, 0xCA, 0x01);
    //CyDelayUs(500);
    vTaskDelay(0.05 / portTICK_PERIOD_MS);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-6
 * Data=0x00 is set in Address=0xCA.
 * Vpp terminal is grounded.
 */
void e_fuse_stage10_6_1()
{
    DS_write(GP2Y0E, 0xCA, 0x00);
    //VDAC8_SetValue(0);
    //_vpp_off();
   gpio_set_level(VDD_PIN, 0);

}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-1'
 * Data=0xFF is set in Address=0xEC.
 * 3.3V is applied in Vpp terminal.
 */
void e_fuse_stage10_1_2()
{

    DS_write(GP2Y0E, 0xEC, 0xFF);
    //VDAC8_SetValue(206);
    //_vpp_on();
    gpio_set_level(VDD_PIN, 1);

}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-2'
 * Data=0x3F is set in Address=0xC8.
 */
void e_fuse_stage10_2_2()
{

    DS_write(GP2Y0E, 0xC8, 0x3F);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-3'
 * Data=0x04 is set in Address=0xC9.
 */
void e_fuse_stage10_3_2()
{

    DS_write(GP2Y0E, 0xC9, 0x04);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-4'
 * Data=0x01 is set in Address=0xCD.
 */
void e_fuse_stage10_4_2()
{

    DS_write(GP2Y0E, 0xCD, 0x01);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-5'
 * Data=0x01 is set in Address=0xCA.
 * Wait for 500us.
 */
void e_fuse_stage10_5_2()
{

    DS_write(GP2Y0E, 0xCA, 0x01);
    //CyDelayUs(500);
    vTaskDelay(0.05 / portTICK_PERIOD_MS);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-6'
 * Data=0x00 is set in Address=0xCA.
 * Vpp terminal is grounded.
 */
void e_fuse_stage10_6_2()
{

    DS_write(GP2Y0E, 0xCA, 0x00);
    //VDAC8_SetValue(0);
    gpio_set_level(VDD_PIN, 0);

}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-7
 * Data=0x00 is set in Address=0xEF.
 * Data=0x40 is set in Address=0xC8.
 * Data=0x00 is set in Address=0xC8.
 */
void e_fuse_stage10_7()
{
    DS_write(GP2Y0E, 0xEF, 0x00);
    DS_write(GP2Y0E, 0xC8, 0x40);
    DS_write(GP2Y0E, 0xC8, 0x00);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-8
 * Data=0x06 is set in Address=0xEE.
 */
void e_fuse_stage10_8()
{

    DS_write(GP2Y0E, 0xEE, 0x06);
}

/*
 * (Fig.40 E-Fuse Program Flow)
 * Stage 10-9
 * Data=0xFF is set in Address=0xEC.
 * Data=0x03 is set in Address=0xEF.
 * Read out the data in Address=0x18 and Address=0x19.
 */
void e_fuse_stage10_9()
{
    //char buffer[255] = {};
    DS_write(GP2Y0E, 0xEC, 0xFF);
    DS_write(GP2Y0E, 0xEF, 0x03);
     uint8_t bit_replacemnt_18 = DS_read(GP2Y0E, 0x18);
     uint8_t bit_replacemnt_19 = DS_read(GP2Y0E, 0x19);
    //sprintf(buffer, "Check 0x18 => %d\r\n", bit_replacemnt_18);
    printf("Check 0x18 => %d\r\n", bit_replacemnt_18);
    //sprintf(buffer, "Check 0x19 => %d\r\n", bit_replacemnt_19);
    printf("Check 0x19 => %d\r\n", bit_replacemnt_19);
    if (bit_replacemnt_18 == 0x82 && bit_replacemnt_19 == 0x00)
    {
        //sprintf(buffer, "Bit Replacement (stage 10) is SUCCESSFUL\r\n");
        printf("Bit Replacement (stage 10) is SUCCESSFUL\r\n");
    }
    else
    {
       // sprintf(buffer, "Bit Replacement (stage 10) is FAILURE\r\n");
        printf( "Bit Replacement (stage 10) is FAILURE\r\n");
    }
}

void Ds_change(uint8_t new_address)
{
    char buffer[255] = {};
    if (new_address == GP2Y0E)
    {
        printf("[ERROR] The new address must be other than 0x08!\r\n");
        return;
    }
    if (new_address > 0x0f)
    {
        printf("[ERROR] The new address must be 0x0f or lower!\r\n");
        return;
    }
    ///VDAC8_SetValue(0);
    gpio_set_level(VDD_PIN, 0);

    //CyDelayUs(500);
    vTaskDelay(0.05 / portTICK_PERIOD_MS);
    e_fuse_stage1();
    printf("stage1\r\n");
    e_fuse_stage2();
    printf("stage2\r\n");
    e_fuse_stage3();
    printf("stage3\r\n");
    e_fuse_stage4(new_address);
    printf("stage4\r\n");
    e_fuse_stage5();
    printf("stage5\r\n");
    e_fuse_stage6();
    printf("stage6\r\n");
    e_fuse_stage7();
    printf("stage7\r\n");
    e_fuse_stage8();
    printf("stage8\r\n");
    const uint8_t result = e_fuse_stage9();
    sprintf(buffer, "e_fuse_stage9() result => %x\r\n", result); //lo codifica en ascci
    if (result)
    {
        e_fuse_stage10_1_1();
        e_fuse_stage10_2_1();
        e_fuse_stage10_3_1();
        e_fuse_stage10_4_1();
        e_fuse_stage10_5_1();
        e_fuse_stage10_6_1();
        e_fuse_stage10_1_2();
        e_fuse_stage10_2_2();
        e_fuse_stage10_3_2();
        e_fuse_stage10_4_2();
        e_fuse_stage10_5_2();
        e_fuse_stage10_6_2();
        e_fuse_stage10_7();
        e_fuse_stage10_8();
        e_fuse_stage10_9();
    }

    printf(buffer);
}
 
float DS_get_data(char sladress)
{
    
     int distance_cm,shift_dist, datai2c[2] = {0, 0};
    float distance_cm2;
  
    datai2c[0] = DS_read(sladress, DISTANCE_ADDR1);
    datai2c[1] = DS_read(sladress, DISTANCE_ADDR2);
    shift_dist = DS_read(sladress, 0x35);
    distance_cm2 = (datai2c[0] * 16 + datai2c[1]) /16/2^shift_dist; //calculo de distancia
      //(((uint16_t) data[0] << 4) + (data[1] & 0x0f)) / 16 / 4;

          /*  printf("*****I2C distancia******\n");
            printf("sensor_addr: %f\n", distance_cm2);
            printf("*******************\n");*/
    return distance_cm2;
    
}
void DS_range(char adress, char distance)
{
    //DS_write( adress,  SHIFT_ADDR,  0x01);

    if (distance == 0x01)// 128cm
    {
    DS_write( adress,  SHIFT_ADDR,  0x01);
    }
    else// 64 cm
    {
    DS_write( adress,  SHIFT_ADDR,  0x02);
    }
 
} 