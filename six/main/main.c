//assignment 6: Save log messages beyond a reboot and make these log messages accessible
// + log_buffer is already implemented in RTC fast memory. Add restart_counter that
// + persistently stores the number of times esp32 has been booted
// Save the logs into a file in flash via SPIFFS, and make this file accesible by
// + using a web server (that can be accessed from computer) to display it

//code adapted from esp32 http file server example

#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/rtc.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp32/rom/uart.h"
#include "log_buffer.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "soc/soc_caps.h"
#if SOC_SDMMC_HOST_SUPPORTED
#include "driver/sdmmc_host.h"
#endif

/* This example demonstrates how to create file server
 * using esp_http_server. This file has only startup code.
 * Look in file_server.c for the implementation */

#define MOUNT_POINT "/sdcard"
static const char *TAG="example";
const char tag[20]="LOG_OVER_REBOOT";
const char TAGs[10] = "SPIFFS";
int bootCount = 0;

/* ESP32-S2/C3 doesn't have an SD Host peripheral, always use SPI,
 * ESP32 can choose SPI or SDMMC Host, SPI is used by default: */

#ifndef CONFIG_EXAMPLE_USE_SDMMC_HOST
#define USE_SPI_MODE
#endif
// DMA channel to be used by the SPI peripheral
#if CONFIG_IDF_TARGET_ESP32
#define SPI_DMA_CHAN    1
// on ESP32-S2, DMA channel must be the same as host id
#elif CONFIG_IDF_TARGET_ESP32S2
#define SPI_DMA_CHAN    host.slot
#elif CONFIG_IDF_TARGET_ESP32C3
// on ESP32-C3, DMA channels are shared with all other peripherals
#define SPI_DMA_CHAN    1
#endif //CONFIG_IDF_TARGET_ESP32

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
static sdmmc_card_t* mount_card = NULL;
static char * mount_base_path = MOUNT_POINT;
#endif
#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#if CONFIG_IDF_TARGET_ESP32C3
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   10
#else
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13
#endif // CONFIG_IDF_TARGET_ESP32C3
#endif //USE_SPI_MODE

/* Function to initialize SPIFFS */
static esp_err_t init_spiffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,   // This decides the maximum number of files that can be created on the storage
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}

/* Declare the function which starts the file server.
 * Implementation of this function is to be found in
 * file_server.c */
esp_err_t start_file_server(const char *base_path);
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
void sdcard_mount(void)
{
    /*sd_card part code*/
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

#ifndef USE_SPI_MODE
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    // slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
#else
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        ESP_ERROR_CHECK(ret);
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    mount_card = card;
#endif //USE_SPI_MODE
    if(ret != ESP_OK){
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        ESP_ERROR_CHECK(ret);
    }
    sdmmc_card_print_info(stdout, card);

}

static esp_err_t unmount_card(const char* base_path, sdmmc_card_t* card)
{
#ifdef USE_SPI_MODE
    esp_err_t err = esp_vfs_fat_sdcard_unmount(base_path, card);
#else
    esp_err_t err = esp_vfs_fat_sdmmc_unmount();
#endif
    ESP_ERROR_CHECK(err);
#ifdef USE_SPI_MODE
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    err = spi_bus_free(host.slot);
#endif
    ESP_ERROR_CHECK(err);

    return err;
}

#endif //CONFIG_EXAMPLE_MOUNT_SD_CARD

void periodicRestart(void *pvParameter){
    uint32_t period; 

    period=esp_random()%10+2;
    vTaskDelay(1000*period / portTICK_RATE_MS);
    esp_restart();
}

void app_main(void)
{

    /*Mount the SDcard first if needed.*/
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
    sdcard_mount();
#endif
    //benchmarking
    struct timespec nvsTimeStart;
    clock_gettime(CLOCK_MONOTONIC, &nvsTimeStart);
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("NVS handle opened\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                ESP_LOGI("RESTART", "Restart counter = %d", restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        bootCount = restart_counter;
        // Write
        printf("Updating restart counter in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }
    struct timespec nvsTimeEnd;
    clock_gettime(CLOCK_MONOTONIC, &nvsTimeEnd);
    double nvsTime = nvsTimeEnd.tv_sec - nvsTimeStart.tv_sec + 1e-9 * (nvsTimeEnd.tv_nsec - nvsTimeStart.tv_nsec);
    printf("NVS runtime: %f seconds\n", nvsTime);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    esp_log_set_vprintf(&bufferAppend);
    if (rtc_get_reset_reason(0) == POWERON_RESET){
		ESP_LOGI(tag, "reset reason: %d\n",rtc_get_reset_reason(0));
		bufferInit();
	} else if (rtc_get_reset_reason(0) != DEEPSLEEP_RESET) {
		ESP_LOGI(tag, "reset reason: %d\n",rtc_get_reset_reason(0));
        printBuffer();
	} else {
        printBuffer();
    }
    /* Initialize file storage */
    struct timespec spiTimeStart;
    clock_gettime(CLOCK_MONOTONIC, &spiTimeStart);
    ESP_ERROR_CHECK(init_spiffs());
    ESP_LOGI(TAGs, "Opening file");
    FILE* f = fopen("/spiffs/logs.txt", "w+");
    if (f == NULL) {
        ESP_LOGE(TAGs, "Failed to open file for writing");
        return;
    }
    fprintf(f, "%s\n", getBuffer());
    fclose(f);
    ESP_LOGI(TAGs, "Buffer stored in logs");
    struct timespec spiTimeEnd;
    clock_gettime(CLOCK_MONOTONIC, &spiTimeEnd);
    double spiTime = spiTimeEnd.tv_sec - spiTimeStart.tv_sec + 1e-9 * (spiTimeEnd.tv_nsec - spiTimeStart.tv_nsec);
    printf("SPIFFS runtime: %f seconds\n", spiTime);

    /* Start the file server */
    ESP_ERROR_CHECK(start_file_server("/spiffs"));

    xTaskCreate(&periodicRestart, "periodicRestart", 2048,NULL,5,NULL );

    int i=0;
    while (1){
		ESP_LOGI(tag, "%d: waiting %d secs",bootCount,i);
        i++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
#ifdef CONFIG_EXAMPLE_MOUNT_SD_CARD
    //deinitialize the bus after all devices are removed
    ESP_ERROR_CHECK(unmount_card(mount_base_path, mount_card));
#endif
}
