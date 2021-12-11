//assignment 2: create a task reading frequency from the serial monitor and storing it in the global variable
//  + create a monitoring task that outputs the current value of the number of the frequency periodically on the OLED
//assignment 3: add a buffer on the stack to save changes of the frequency and the number of seconds since starting
// + print all buffer elements when the buffer is full

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "ssd1306.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define LED GPIO_NUM_16
#define EX_UART_NUM UART_NUM_0
static const int uart_buf_size = 1024;
static QueueHandle_t uart0_queue;
int readValue = 500;
int changeIndex = 0;
//Assignment 3: Allocating buffer on the stack to save frequency changes
//buffer to store frequency values
int changes[4];
//buffer to store number of seconds since start
int64_t seconds[4];

void frequency(char* freqString){
    long temp = 0;
    char* endptr;
    temp = strtol(freqString, &endptr, 10);
    if(endptr == freqString || temp < 0){
        //if input is not a number
        printf("Invalid input\n");
        ssd1306_clearScreen();
	    ssd1306_printFixedN(0, 24, "Invalid", STYLE_NORMAL,1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
	    ssd1306_clearScreen();
        return;
    }
    //cast to int, assume that the frequency value can be stored as int
    int freq = (int) temp;
    readValue = freq;
    ssd1306_clearScreen();
	ssd1306_printFixedN(0, 24, freqString, STYLE_NORMAL,2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
	ssd1306_clearScreen();
    changes[changeIndex] = readValue;
    //timer code referenced from https://github.com/espressif/esp-idf/blob/master/examples/system/esp_timer/main/esp_timer_example_main.c
    //use esp_timer_get_time, divide by 1000000 to convert from microseconds to seconds.
    seconds[changeIndex] = esp_timer_get_time() / 1000000;
    changeIndex++;
    if(changeIndex == 4){
        printf("Changes in frequency:\n");
        for(int i = 0; i < 4; i++){
            printf("Frequency set to: %d Hz, changed at %" PRId64 " seconds\n", changes[i], seconds[i]);
        }
        changeIndex = 0;
    }
}

static void UartEventTask(void *_)
{
    uart_event_t event;
    uint8_t *read_buf = (uint8_t *)malloc(uart_buf_size);
    int size;
    memset(read_buf, 0, uart_buf_size);
    while (1)
    {
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_PATTERN_DET:
                memset(read_buf, 0, uart_buf_size);
                //handle variable input length by using uart_pattern_pop_pos
                size = uart_pattern_pop_pos(EX_UART_NUM);
                //If pos return -1, refresh ring buffer and request for input again
                if(size == -1){
                    ESP_LOGI(TAG, "Overflow: flushing buffer. Input desired frequency again.");
                    uart_flush_input(EX_UART_NUM);
                } else {
                    uart_read_bytes(EX_UART_NUM, read_buf, size, pdMS_TO_TICKS(100));
                    //read \n to prevent error on terminal
                    uint8_t nextLine[1];
                    uart_read_bytes(EX_UART_NUM, nextLine, 1, pdMS_TO_TICKS(100));
                    ESP_LOGI(TAG, "Input Frequency: %s", read_buf);
                    frequency((char *) read_buf);
                }
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d size %u", event.type, event.size);
                break;
            }
        }
    }
    //free to prevent memory leak
    free(read_buf);
}

static void UartInit(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, uart_buf_size * 2, uart_buf_size * 2, 20, &uart0_queue, 0);
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '\n', 1, 9, 0, 0);
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '\r', 1, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);
}

void BlinkLed(void *_)
{
    gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    while (1)
    {
        gpio_set_level(LED, 0);
        vTaskDelay(readValue / portTICK_PERIOD_MS);
        gpio_set_level(LED, 1);
        vTaskDelay(readValue / portTICK_PERIOD_MS);
    }
}

void initOLED(){
    char str[50];

    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_128x64_i2c_init();
    ssd1306_clearScreen();
	snprintf(str,49,"Hallo");
	ssd1306_printFixedN(0, 24, str, STYLE_NORMAL,2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
	ssd1306_clearScreen();
}


void app_main(void)
{
    UartInit();
    initOLED();
    //Create a task to handler UART event from ISR
    xTaskCreate(UartEventTask, "UartMonitor", 2048, NULL, 12, NULL);
    xTaskCreate(BlinkLed, "Blink", 1024, NULL, 1, NULL);
    /*Add the monitoring task, that outputs the set interval*/

}
