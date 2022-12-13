

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include <driver/ledc.h>
#include <driver/adc.h>
#include "driver/mcpwm.h"
#include <driver/adc.h>
#include "freertos/queue.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>


#define WINDOWS         14
#define DOORS           32
#define HIGH            1
#define LOW             0

#define UART_NUM        UART_NUM_2
#define BUF_SIZE        1024
#define TASK_MEMORY     1024
#define MAX_DUTY        3000 /* max duty cycle */
#define MIN_DUTY        1000

#define FRONT_LT        18
#define BACK_LT         19
#define SIDE_LT         22   
#define GARAGE_LT       2

QueueHandle_t ADCQueue;

SemaphoreHandle_t OPEN = NULL;
SemaphoreHandle_t CLOSE = NULL;

// You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH_US (1000) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2000) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        (90)   // Maximum angle in degree upto which servo can rotate
#define SERVO_MIN_DEGREE        (0)

#define SERVO_PULSE_GPIO        (DOORS)   // GPIO connects to the PWM signal line

int i = 0;
int w = 0;


static inline uint32_t example_convert_servo_angle_to_duty_us(int angle)
{
    return (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}


void Doors_task(void){

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO); // To drive a RC servo, one MCPWM generator is enough

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);


}

/* ADC setup */
void adc_setup(void)
{
    /*Set the ADC with @ 12 bits -> 2^12 = 4096*/
    adc1_config_width(ADC_WIDTH_BIT_12);
 
    /*Set CHANNEL 6 @ 2600 mV*/
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); 
}

    

static void gpio(void)
{

// LIGHTS GPIO

gpio_pad_select_gpio(FRONT_LT); /* select GPIO pins */
gpio_set_direction(FRONT_LT, GPIO_MODE_OUTPUT); /* set as output */

gpio_pad_select_gpio(BACK_LT); /* select GPIO pins */
gpio_set_direction(BACK_LT, GPIO_MODE_OUTPUT); /* set as output */

gpio_pad_select_gpio(SIDE_LT); /* select GPIO pins */
gpio_set_direction(SIDE_LT, GPIO_MODE_OUTPUT); /* set as output */

gpio_pad_select_gpio(GARAGE_LT); /* select GPIO pins */
gpio_set_direction(GARAGE_LT, GPIO_MODE_OUTPUT); /* set as output */


}


void pwm1_setup(void){
/*
Set LEDC Timer:
                5000hz
                AUTO_CLK
                TIMER_1
                LOW_SPEED
                13 BIT
*/
ledc_timer_config_t timerConfig;
timerConfig.duty_resolution = LEDC_TIMER_13_BIT; 
timerConfig.timer_num = LEDC_TIMER_1;
timerConfig.freq_hz = 5000; 
timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
timerConfig.clk_cfg = LEDC_AUTO_CLK;
ledc_timer_config(&timerConfig);

/*
Set LEDC Channel:
                GPIO_21
                LOW_SPEED
                TIMER_1
                INTERRUPT_DISABLE
                0 duty
                Max duty 
*/
ledc_channel_config_t channelConfig;
channelConfig.gpio_num = WINDOWS;    
channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
channelConfig.channel = LEDC_CHANNEL_1;
channelConfig.intr_type = LEDC_INTR_DISABLE;
channelConfig.timer_sel = LEDC_TIMER_1;
channelConfig.duty = 0;
channelConfig.hpoint = MAX_DUTY;   
ledc_channel_config(&channelConfig);
}




static void uart_init()
{
uart_config_t uart_config = {
.baud_rate = 9600,
.data_bits = UART_DATA_8_BITS,
.parity    = UART_PARITY_DISABLE,
.stop_bits = UART_STOP_BITS_1,
.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
.source_clk = UART_SCLK_APB,
};


// Installing UART driver, setting pins and configuration
uart_param_config(UART_NUM, &uart_config);
uart_set_pin(UART_NUM, 5, 4, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);

}

void ledc_ON(void){

/* Iterate over channel 1 duty cycle, increase */
for (i = 0; i < MAX_DUTY; i += 10){ 
    /* set duty cycle */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,i);
    /* update duty cycle */
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1); //update duty 
    vTaskDelay(10/portTICK_PERIOD_MS); /* 10ms */ 

     }

}

void ledc_OFF(void){

//     /* Iterate over channel 1 duty cycle, decrease */
for (i = MAX_DUTY; i > 0; i -= 10){ 
    /* set duty cycle */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,i);
    /* update duty cycle */
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1); //update duty 
    vTaskDelay(10/portTICK_PERIOD_MS); /* 10ms */ 


}


}



static void uart_task(void *arg)
{
uint8_t *data = (uint8_t *) malloc(BUF_SIZE);



while(1){
bzero(data,BUF_SIZE);

int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));

for(size_t i = 0; i < len; i++)
{
    char value = data[i];

    switch (value)
    {
    case 'q':
        gpio_set_level(FRONT_LT, HIGH);
        break;
    
    case 'w':
        gpio_set_level(FRONT_LT, LOW);
        break;
    
    case 'e':
        gpio_set_level(BACK_LT, HIGH);
        break;

    case 'r':
        gpio_set_level(BACK_LT, LOW);
        break;

    case 't':
        gpio_set_level(SIDE_LT, HIGH);
        break;

    case 'y':
        gpio_set_level(SIDE_LT, LOW);
        break;

    case 'u':
        gpio_set_level(GARAGE_LT, HIGH);
        break;

    case 'i':
        gpio_set_level(GARAGE_LT, LOW);
        break;
    
    case 'o':
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, SERVO_MAX_PULSEWIDTH_US );
            vTaskDelay(pdMS_TO_TICKS(10)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
        //    xSemaphoreGive(OPEN);

        break;

    case 'p':

            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, SERVO_MIN_PULSEWIDTH_US);
            vTaskDelay(pdMS_TO_TICKS(10)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
        //    xSemaphoreGive(CLOSE);


        break;

    case 'a':

    ledc_ON();


        break;

    case 's':

   ledc_OFF();

        break;
    
    default:

        break;
    
    }


free(data);

}
}
}

void PWMtask(void *pvParameters)
{
   int storeData;

   char* test1_str;
    while(1){
        if(xQueueReceive(ADCQueue, &storeData, (TickType_t)100) == pdPASS){

            printf("Data receive from task 1: %d\n", storeData); /* display data receive */
            vTaskDelay(100/portTICK_PERIOD_MS); /* 100 ms */ 
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,storeData);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1); //update duty 

        }

        if(storeData < 500){
            test1_str = "LOW \n";
             uart_write_bytes(UART_NUM, (const char*)test1_str, strlen(test1_str));

        }

        if((storeData >= 500) && (storeData <= 3000) ){
            test1_str = "MED \n";
            uart_write_bytes(UART_NUM, (const char*)test1_str, strlen(test1_str));

        }

        if(storeData > 3000 ){
            test1_str = "HIGH \n";
            uart_write_bytes(UART_NUM, (const char*)test1_str, strlen(test1_str));
}
        

    }

}

void Open(void *pvParameters)
{
    int angle = -SERVO_MAX_DEGREE;

    while(1){
        if(xSemaphoreTake(OPEN, 100/portTICK_RATE_MS) == pdTRUE){ 

            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(angle));
            vTaskDelay(pdMS_TO_TICKS(10)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
        }
    }    
}

void Close(void *pvParameters)
{
    int angle = -SERVO_MAX_DEGREE;

    while(1){

        if(xSemaphoreTake(CLOSE, 100/portTICK_RATE_MS) == pdTRUE){ 
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(angle));
            vTaskDelay(pdMS_TO_TICKS(10)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
        }
    }    
}

void ADCtask(void *pvParameters)
{
    while (1)
    {
         /* Read adc value @ CHANNEL 6*/
        int val = adc1_get_raw(ADC1_CHANNEL_6);
        // char send = (val/4095) * 100;   
        vTaskDelay(100/portTICK_PERIOD_MS); /* 100 ms */
        /* Print reading */
        printf("ADC val: %d\n", val);
        xQueueSendToBack(ADCQueue, &val, 0);

    }
}



void app_main(void)
{

// example_convert_servo_angle_to_duty_us();

pwm1_setup();
//pwm2_setup();
gpio();
uart_init();
Doors_task();

ADCQueue = xQueueCreate(5, sizeof(int));

OPEN = xSemaphoreCreateBinary(); 
CLOSE = xSemaphoreCreateBinary(); 



xTaskCreate(uart_task, "uart_task", TASK_MEMORY, NULL, 5, NULL);
xTaskCreate(&ADCtask, "ADCtask", 2048, NULL, 5, NULL);
xTaskCreate(&PWMtask, "PWMtask", 2048, NULL, 5, NULL);
xTaskCreate(&Open, "Open", 2048, NULL, 5, NULL);
xTaskCreate(&Close, "Close", 2048, NULL, 5, NULL);

}
