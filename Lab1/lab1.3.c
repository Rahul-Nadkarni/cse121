#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h" 

#define LED_GPIO_PIN GPIO_NUM_7 


void blink_task(void *pvParameter){
	esp_rom_gpio_pad_select_gpio(LED_GPIO_PIN); 
	gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT); 

	while(1){
		gpio_set_level(LED_GPIO_PIN, 1); 
		vTaskDelay(pdMS_TO_TICKS(500)); 
		gpio_set_level(LED_GPIO_PIN, 0); 
		vTaskDelay(pdMS_TO_TICKS(500)); 
	}
}
void app_main(void)
{
	xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL); 
}
