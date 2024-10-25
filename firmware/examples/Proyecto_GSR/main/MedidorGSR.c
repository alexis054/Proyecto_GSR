/*! @mainpage Ejemplo Bluetooth - Filter
 *
 * @section genDesc General Description
 *
 * This section describes how the program works.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN_X	 	| 	GPIO_X		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 12/09/2023 | Document creation		                         |
 *
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "neopixel_stripe.h"
#include "ble_mcu.h"
#include "timer_mcu.h"

#include "iir_filter.h"

#include <uart_mcu.h>
#include <analog_io_mcu.h>
#include <switch.h>

/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define LED_BT	            LED_1
#define T_SENIAL            4000 
#define CHUNK               4 

#define CONFIG_MEASURE_PERIOD 2000
#define BAUD_RATE 115200
#define GSR_TRESHOLD 350
/*==================[internal data definition]===============================*/

TaskHandle_t covert_digital_task_handle = NULL;
TaskHandle_t turnon_LEDs_GSR_task_handle = NULL;
TaskHandle_t fft_task_handle = NULL;

uint16_t reading = 0;
bool measure_reading = false;
bool filter = false;

/*==================[internal functions declaration]=========================*/

void FuncTimerA(void* param){

    vTaskNotifyGiveFromISR(covert_digital_task_handle, pdFALSE);
	vTaskNotifyGiveFromISR(turnon_LEDs_GSR_task_handle, pdFALSE); 

}



void FuncTimerSenial(void* param){
    xTaskNotifyGive(fft_task_handle);
}

void read_data(uint8_t * data, uint8_t length)
{
    switch(data[0]){
        case 'A':
            filter = true;
            break;
        case 'a':
            filter = false;
            break;
    }
}
static void FftTask(void *pvParameter)
{
    char msg[128];
    char msg_chunk[24];
    static uint8_t indice = 0;
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(filter){
            HiPassFilter(&ecg[indice], ecg_filt, CHUNK);
            LowPassFilter(ecg_filt, ecg_filt, CHUNK);
        } else{
            memcpy(ecg_filt, &ecg[indice], CHUNK*sizeof(float));
        }
        strcpy(msg, "");
        for(uint8_t i=0; i<CHUNK; i++){
            sprintf(msg_chunk, "*G%.2f*", ecg_filt[i]);
            strcat(msg, msg_chunk);
        }
        indice += CHUNK;

        BleSendString(msg);
    }
}

static void turnon_LEDs_GSR(void *pvParameter)
{ 
	while(true)
	{	
		if (measure_reading)
			{
				if(reading < GSR_TRESHOLD)
				{	
                    LedOn(LED_1);
                }
				else {
						LedToggle(LED_2);
						LedOff(LED_1);
					 }
					
			}
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}	
}

void static read_switches(void *pvParameter)
{
    bool *flags = (bool*) pvParameter;
	*flags = !*flags;
}

/*==================[external functions definition]==========================*/
void app_main(void){
    uint8_t blink = 0;
    static neopixel_color_t color;
    ble_config_t ble_configuration = {
        "ESP_EDU_1",
        read_data
    };
    timer_config_t timer_senial = {
        .timer = TIMER_B,
        .period = T_SENIAL*CHUNK,
        .func_p = FuncTimerSenial,
        .param_p = NULL
    };

    NeoPixelInit(BUILT_IN_RGB_LED_PIN, BUILT_IN_RGB_LED_LENGTH, &color);
    NeoPixelAllOff();
    TimerInit(&timer_senial);
    LedsInit();  
    LowPassInit(CONFIG_MEASURE_PERIOD, 30, ORDER_2);
    HiPassInit(CONFIG_MEASURE_PERIOD, 1, ORDER_2);
    BleInit(&ble_configuration);

    xTaskCreate(&FftTask, "FFT", 4096, NULL, 5, &fft_task_handle);
    TimerStart(timer_senial.timer);

 
    
    analog_input_config_t analog_input ={
		.input = CH1,
		.mode = ADC_SINGLE,
	};
	
	AnalogInputInit(&analog_input);
	AnalogOutputInit();

	serial_config_t serial_pc ={
		.port = UART_PC,
		.baud_rate = BAUD_RATE,
		.func_p = NULL,
		.param_p = NULL,
	};

	UartInit(&serial_pc);

	timer_config_t timer_measurement = {
    	.timer = TIMER_A,
        .period = CONFIG_MEASURE_PERIOD,
        .func_p = FuncTimerA,
        .param_p = NULL
    };

	LedsInit();
	SwitchesInit();

	SwitchActivInt(SWITCH_1, read_switches, &measure_reading);
	SwitchActivInt(SWITCH_2, LedsOffAll, NULL);

	TimerInit(&timer_measurement);

	xTaskCreate(&convert_digital, "Convertir señal a Digital", 1024, NULL, 5, &covert_digital_task_handle);
	xTaskCreate(&turnon_LEDs_GSR, "Prender los LEDs segun los eventos", 1024, NULL, 5, &turnon_LEDs_GSR_task_handle);

	TimerStart(timer_measurement.timer);
}

/*==================[end of file]============================================*/
