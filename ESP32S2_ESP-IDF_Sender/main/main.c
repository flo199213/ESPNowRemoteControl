/**
 * ESP32 ESPNow Sender
 *
 * @author    Florian Staeblein
 * @date      2024/07/29
 * @copyright © 2024 Florian Staeblein
 * 
 * ==============================================================
 * 
 * Configuration Wemos S2 Mini:
 * - Board: "ESP32S2 Dev Module"
 * - CPU Frequency: "240MHz (WiFi)"
 * - USB CDC On Boot: "Enabled"   <------------ Important!
 * - USB DFU On Boot: "Disabled"
 * - USB Firmware MSC On Boot: "Disabled"
 * - Flash Size: "4Mb (32Mb)"
 * - Partition Scheme: "No OTA (2MB APP/2MB SPIFFS)"
 * - PSRAM: "Enabled"
 * - Upload Mode: "Internal USB"
 * - Upload Speed: "921600"
 * 
 * -> Leave everything else on default!
 * 
 * Important notice:
 * "USB CDC On Boot" flag must be set at all times. This flag
 * causes the Wemos S2 Mini to report as a COM interface immediately
 * after booting via USB. This means that the microcontroller can
 * be programmed WITHOUT having to press the "BOOT" and "RESET"
 * buttons again.
 * 
 * ==============================================================
 */

//===============================================================
// Includes
//===============================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include <driver/gpio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

static const char *TAG = "ESPNow Sender";

//===============================================================
// Defines
//===============================================================
#define PIN_BUTTON1                 3     // GPIO 3
#define PIN_BUTTON2                 37    // GPIO 37
#define PIN_BUTTON3                 11    // GPIO 11
#define PIN_BUTTON4                 16    // GPIO 16
#define PIN_LED_GND                 35    // GPIO 35
#define PIN_LED_VCC                 5     // GPIO 5
#define PIN_KEEPALIVE               7     // GPIO 7

// Destination MAC address
#define MY_RECEIVER_MAC             { 0x84, 0xfc, 0xe6, 0xd1, 0x21, 0x94 }

// Time between ESPNow retransmission
#define RETRANSMISSION_TIME_MS      1000

// Debug output
#define Serial.print printf

//===============================================================
// Global Variables
//===============================================================
typedef struct __attribute__((packed))
{
    uint8_t buttons;
} exchange_struct_t;

static exchange_struct_t data;
static EventGroupHandle_t s_evt_group;

//===============================================================
// Will be called on ESPNow packet send
//===============================================================
static void packet_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    assert(status == ESP_NOW_SEND_SUCCESS || status == ESP_NOW_SEND_FAIL);
    xEventGroupSetBits(s_evt_group, BIT(status));
}

//===============================================================
// Initializes the ESPNow sender
//===============================================================
static void init_espnow_slave(void)
{
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84)); // 21dBm (Max TX power)
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(packet_sent_cb));

    // Alter this if you want to specify the gateway mac, enable encyption, etc
    const esp_now_peer_info_t broadcast_destination = {
        .peer_addr = MY_RECEIVER_MAC,
        .channel = 1,
        .ifidx = ESP_IF_WIFI_STA
    };
    ESP_ERROR_CHECK( esp_now_add_peer(&broadcast_destination) );
}

//===============================================================
// Deinitializes the ESPNow sender
//===============================================================
static void deinit_espnow_slave(void)
{
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_stop());
}

//===============================================================
// Sends an ESPNow frame
//===============================================================
static esp_err_t send_espnow_data(void)
{
    const uint8_t destination_mac[] = MY_RECEIVER_MAC;

    // Set LED on
    gpio_set_level(PIN_LED_VCC, true);

    // Reset Feedback event
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

    // Send ESPNow frame
    //ESP_LOGI(TAG, "Sending %u bytes to "  + MACSTR, sizeof(data), MAC2STR(destination_mac));
    esp_err_t err = esp_now_send(destination_mac, (uint8_t*)&data, sizeof(data));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error (%d)", err);
        return ESP_FAIL;
    }

    // Wait for callback function to set status bit
    EventBits_t bits = xEventGroupWaitBits(s_evt_group, BIT(ESP_NOW_SEND_SUCCESS) | BIT(ESP_NOW_SEND_FAIL), pdTRUE, pdFALSE, 2000 / portTICK_PERIOD_MS);
    if (!(bits & BIT(ESP_NOW_SEND_SUCCESS)))
    {
        if (bits & BIT(ESP_NOW_SEND_FAIL))
        {
            ESP_LOGE(TAG, "Send error");
            return ESP_FAIL;
        }
        ESP_LOGE(TAG, "Send timed out");
        return ESP_ERR_TIMEOUT;
    }

    // Set LED off
    gpio_set_level(PIN_LED_VCC, false);

    ESP_LOGI(TAG, "Sent!");
    return ESP_OK;
}

//===============================================================
// Reads button input state
//===============================================================
void GetButtonsState()
{ 
    // Get button input states
    data.buttons = (uint8_t)(gpio_get_level(PIN_BUTTON4) << 3 | gpio_get_level(PIN_BUTTON3) << 2 | gpio_get_level(PIN_BUTTON2) << 1 | gpio_get_level(PIN_BUTTON1));
}

//===============================================================
// Main task
//===============================================================
int app_main()
{
    // Set latch first
    gpio_reset_pin(PIN_KEEPALIVE);
    gpio_set_direction(PIN_KEEPALIVE, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_KEEPALIVE, true);

    // Initialize button inputs with pulldown
    gpio_reset_pin(PIN_BUTTON1);
    gpio_reset_pin(PIN_BUTTON2);
    gpio_reset_pin(PIN_BUTTON3);
    gpio_reset_pin(PIN_BUTTON4);
    gpio_set_direction(PIN_BUTTON1, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_BUTTON2, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_BUTTON3, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_BUTTON4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON1, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(PIN_BUTTON2, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(PIN_BUTTON3, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(PIN_BUTTON4, GPIO_PULLDOWN_ONLY);

    // Get Button input states as fast as possible
    GetButtonsState();
    
    // Initialize LED
    gpio_reset_pin(PIN_LED_GND);
    gpio_reset_pin(PIN_LED_VCC);
    gpio_set_direction(PIN_LED_GND, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_VCC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED_GND, false);

    // Keep sending while a button is pressed
    while (data.buttons > 0)
    { 
        // Initialize ESPNow
        init_espnow_slave();

        // Send ESPNow frame
        send_espnow_data();

        // Deinitialize ESPNow
        deinit_espnow_slave();
            
        // Wait 1 second
        vTaskDelay(RETRANSMISSION_TIME_MS / portTICK_PERIOD_MS);

        // Get button input states
        GetButtonsState();
    }

    // Shut off
    gpio_set_level(PIN_KEEPALIVE, false);

    // Wait for shut off
    while (true)
    {
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}