//assignment 1: Develop functionality that allows onboard LED to change colour after pressing a button

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "led_strip.h"

#define CONFIG_BLINK_LED_RMT_CHANNEL 0
#define BLINK_GPIO CONFIG_BLINK_GPIO

static const char *TAG = "BLINK";
//code for configuring LED strip referenced from:
//https://github.com/espressif/esp-idf/tree/master/examples/get-started/blink
//https://www.electronics-lab.com/deep-dive-on-controlling-led-with-esp32-c3-devkitm-1-development-board-using-esp-idf/

//set button as GPIO 15
const int button = 15;
int buttonState = 0;
int lightState = 0;
int color = 18;
int newColor = 16;

void app_main(void){    
	esp_log_level_set("BLINK", ESP_LOG_INFO);       
	ESP_ERROR_CHECK(gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(button, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_set_direction(16, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(17, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(18, GPIO_MODE_OUTPUT));
	
	while(1) {
		//check current state of button
		buttonState = gpio_get_level(button);
		if (buttonState == 1 && lightState == 0){
			ESP_LOGI(TAG, "Turning on the LED");
			//changeCol(color);
			ESP_ERROR_CHECK(gpio_set_level(BLINK_GPIO, 1));
			ESP_ERROR_CHECK(gpio_set_level(color, 1));
			ESP_ERROR_CHECK(gpio_set_level(newColor, 0));
			if(color == 18){
				ESP_LOGI(TAG, "Showing red");
				color = 16;
				newColor = 17;
			} else if(color == 17){
				ESP_LOGI(TAG, "Showing green");
				color = 18;
				newColor = 16;
			} else {
				ESP_LOGI(TAG, "Showing blue");
				color = 17;
				newColor = 18;
			}
			lightState = 1;
		}
		else if (buttonState == 1 && lightState == 1){
			ESP_LOGI(TAG, "Turning off the LED");
			ESP_ERROR_CHECK(gpio_set_level(BLINK_GPIO, 0));
			lightState = 0;
		}
		else {
			
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output low) */
		/*ESP_LOGI(TAG, "Turning on the LED");
		ESP_ERROR_CHECK(gpio_set_level(BLINK_GPIO, 0));
		vTaskDelay(1000 / portTICK_PERIOD_MS);*/
        /* Blink off (output high) */
		/*ESP_LOGI(TAG, "Turning off the LED");
		ESP_ERROR_CHECK(gpio_set_level(BLINK_GPIO, 1));
		vTaskDelay(1000 / portTICK_PERIOD_MS);*/
	}
}

