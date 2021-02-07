#include "esp_log.h"
#include "sdkconfig.h"

#include "GP2Y0E03.h"

#include "I2C.h"

#include "PWM.h"

#include "GPIO.h"

#include "ADC.h"

#include "bluetooth.h"

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle distance_send_evt_queue = NULL;
static xQueueHandle ble_recive_evt_queue = NULL;

static xQueueHandle distance_Strategy_send_evt_queue = NULL;
static xQueueHandle strategy_number_evt_queue = NULL;


static const char *TAG = "i2c-example";

TaskHandle_t taskI2C = NULL;
TaskHandle_t taskADC = NULL;
TaskHandle_t taskPWM = NULL;
TaskHandle_t taskGPIO = NULL;
TaskHandle_t taskStrategy = NULL;

SemaphoreHandle_t print_mux = NULL;

TimerHandle_t xTimer1 = NULL;


typedef enum {
    PARADO,    		/**< carrinho parado.*/
    RETA,        	/**< carrinho andando reto.*/
    ESQUERDA,        	/**< carrinho andando em curva.*/
    DIREITA,
} motor_direction;

typedef enum {
    DESLIGADO,     		
    ATACAR, 
    ESPERAR,       	
} Strategy_number;

typedef struct
{
    motor_direction _motor_direction;
    Strategy_number _Strategy_number;
    Sdistance _Sdistance;

}Strategy;