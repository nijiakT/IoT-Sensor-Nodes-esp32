//assignment 4: connect to wifi and use sntp protocol to get current time
// + print the time on the OLED display. Integrate DHT11 temperature and humidity sensor
// + read both values and output them on the display in float format.
// + integrate BH1750 light sensor and read the lux measurements.
// Data visualisation: using node-red and mqtt, send messages with the current timestamps
// and the three values (temperature, humidity, lux) through mqtt ot node-red.
// + visualise the received values and store the data in a csv file

/*Code adapted from esp sntp example template*/

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <bh1750.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "ssd1306.h"
#include "dht11.h"
#include "mqtt_client.h"

#define ADDR BH1750_ADDR_LO
#define SDA_GPIO 21
#define SCL_GPIO 22

static const char *TAG = "example";
SemaphoreHandle_t xSemaphore;
char* tempStr;
char* humStr;
char* luxStr;
int strBufSize = 64 * sizeof(char);
char strftime_buf[64];
char mqttBuf[1024];
int sntpReady;
int tempReady;
int luxReady;
i2c_dev_t dev;
esp_mqtt_client_handle_t client;

static void obtain_time(void);
static void initialize_sntp(void);
static void initOLED(void);

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

void timeOutput(void *_)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    else {
        // add 500 ms error to the current system time.
        // Only to demonstrate a work of adjusting method!
        {
            ESP_LOGI(TAG, "Add a error for test adjtime");
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            int64_t cpu_time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
            int64_t error_time = cpu_time + 500 * 1000L;
            struct timeval tv_error = { .tv_sec = error_time / 1000000L, .tv_usec = error_time % 1000000L };
            settimeofday(&tv_error, NULL);
        }

            ESP_LOGI(TAG, "Time was set, now just adjusting it. Use SMOOTH SYNC method.");
            obtain_time();
            // update 'now' variable with current time
            time(&now);
    }
#endif
    //set timezone to Germany
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
    while(1){
        struct tm timeinfo = { 0 };
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%R", &timeinfo);
        sntpReady = 1;
    }
}

void dht11Output(void *_)
{
    while(1){
        memset(tempStr, 0, strBufSize);
        memset(humStr, 0, strBufSize);
        if(DHT11_read().status == DHT11_OK){
            snprintf(tempStr,63, "Temperature is %d.%d", DHT11_read().temperature, DHT11_read().tempDec);
            snprintf(humStr,63, "Humidity is %d.%d", DHT11_read().humidity, DHT11_read().humDec);
            tempReady = 1;
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void OLEDtask(void *_){
    while(1){
        if( xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE ){
            ssd1306_clearScreen();
            if(sntpReady == 1){
                ssd1306_printFixedN(0, 10, strftime_buf, STYLE_NORMAL,0);
            }
            if(tempReady == 1){
                ssd1306_printFixedN(0, 20, tempStr, STYLE_NORMAL,0);
                ssd1306_printFixedN(0, 40, humStr, STYLE_NORMAL,0);
            }
            if(luxReady == 1){
                ssd1306_printFixedN(0, 60, luxStr, STYLE_NORMAL,0);
            }
            xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(110));
    }
}

void initSensor()
{
    memset(&dev, 0, sizeof(i2c_dev_t)); // Zero descriptor

    ESP_ERROR_CHECK(bh1750_init_desc(&dev, ADDR, I2C_NUM_1, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH));
}

void lightOutput(void *_)
{
   //from https://github.com/UncleRus/esp-idf-lib/blob/master/examples/bh1750/main/main.c
    uint16_t lux;

    while (1)
    {
        if( xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE )
        {
            while(bh1750_read(&dev, &lux) != ESP_OK){

            }
            luxReady = 1;
            snprintf(luxStr, 63,"Lux is %d", lux);

            xSemaphoreGive( xSemaphore );
        }
        
        vTaskDelay(pdMS_TO_TICKS(110));
    }
}

static void log_error_if_nonzero(const char * message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            esp_mqtt_client_reconnect(client);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://test.mosquitto.org:1883",
    };
    while(sntpReady != 1){
        vTaskDelay(pdMS_TO_TICKS(110));
    }
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void mqttTask(void *_){
    int msg_id = 0;
    while(1){
        memset(mqttBuf, 0, sizeof(mqttBuf));
        if(sntpReady == 1 && tempReady == 1 && luxReady == 1){
            strcat(mqttBuf, "Time ");
            strcat(mqttBuf, strftime_buf);
            strcat(mqttBuf, ",");
            strcat(mqttBuf, tempStr);
            strcat(mqttBuf, ",");
            strcat(mqttBuf, humStr);
            strcat(mqttBuf, ",");
            strcat(mqttBuf, luxStr);
            msg_id = esp_mqtt_client_publish(client, "/assign4sensors", mqttBuf, 0, 0, 0);
            printf("%s\n", mqttBuf);
            ESP_LOGI(TAG, "sent publish time, msg_id=%d", msg_id);
        }
        vTaskDelay(pdMS_TO_TICKS(1100));
    }

}

void app_main(void)
{
    
    tempStr = (char*) malloc(strBufSize);
    humStr = (char*) malloc(strBufSize);
    luxStr = (char*) malloc(strBufSize);
    sntpReady = 0;
    tempReady = 0;
    luxReady = 0;
    i2cdev_init();

    //create freeRTOS Semaphore
    xSemaphore = xSemaphoreCreateMutex();
    if( xSemaphore != NULL )
    {
       printf("Semaphore created\n");
    }
    initSensor();
    initOLED();
    DHT11_init(GPIO_NUM_4);

    xTaskCreate(timeOutput, "time", 4096, NULL, 1, NULL);
    mqtt_app_start();
    xTaskCreate(dht11Output, "dht", 4096, NULL, 1, NULL);
    xTaskCreate(lightOutput, "light", 4096, NULL, 1, NULL);
    xTaskCreate(OLEDtask, "oled", 4096, NULL, 1, NULL);
    xTaskCreate(mqttTask, "mqtt", 4096, NULL, 1, NULL);
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    /**
     * NTP server address could be aquired via DHCP,
     * see LWIP_DHCP_GET_NTP_SRV menuconfig option
     */
#ifdef LWIP_DHCP_GET_NTP_SRV
    sntp_servermode_dhcp(1);
#endif

    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

static void initOLED(){
    char str[50];

    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_128x64_i2c_init();
    ssd1306_clearScreen();
	snprintf(str,49,"IoT");
	ssd1306_printFixedN(0, 24, str, STYLE_NORMAL,2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
	ssd1306_clearScreen();
}
