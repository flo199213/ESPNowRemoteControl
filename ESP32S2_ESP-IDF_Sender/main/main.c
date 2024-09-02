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
#include <limits.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/uart.h>
#include <inttypes.h>
#include <stdbool.h>

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

#include "led_strip.h"

static const char *TAG = "ESPNow Sender";

//===============================================================
// Defines
//===============================================================
#define PIN_KEEPALIVE               7               // GPIO 7  - Keep alive latch
#define PIN_BUTTON1                 37              // GPIO 37 - ESP remote button 1
#define PIN_BUTTON2                 3               // GPIO 3  - ESP remote button 2
#define PIN_BUTTON3                 16              // GPIO 16 - ESP remote button 3
#define PIN_BUTTON4                 11              // GPIO 11 - ESP remote button 4
#define PIN_CHARGING                18              // GPIO 18 - Charging state (Needs pullup, Low = Charging active)

// Not working: !
#define PIN_USBSTATUS               33              // GPIO 33 - USB status

// Defined only for debug reasons
#define PIN_LED                     15              // GPIO 15 - Wemos S2 Mini LED
#define PIN_BUTTON                  0               // GPIO 0  - Wemos S2 Mini Button

// Battery measurement
#define PIN_VBATMESS                9               // GPIO 9  - Battery measurement
#define ADC_CHANNEL_VBATMESS        ADC_CHANNEL_8   // GPIO 9, ADC channel 8

// RGB LED defines
#define PIN_RGB_LED                 5               // GPIO 5  - WS2812B RGB LED
#define NUM_LEDS                    1               // Single RGB LED
#define LED_RMT_CHANNEL             0               // RMT channel 0

// Destination MAC address
#define MY_RECEIVER_MAC             { 0x80, 0x65, 0x99, 0xfa, 0x48, 0x02 }

// Time between ESPNow retransmission
#define RETRANSMISSION_TIME_MS      500

// Bit operations
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

//===============================================================
// Enums
//===============================================================
typedef enum
{
  eCharging,
  eTransmitting,
  eShutOff,
  eIdle,
} SenderState;

//===============================================================
// Global Variables
//===============================================================
// GPIOs
const gpio_config_t gpioKeepAlive = { .pin_bit_mask =  BIT64(PIN_KEEPALIVE), .mode = GPIO_MODE_OUTPUT, .pull_up_en = true, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE };
const gpio_config_t gpioButton1 = { .pin_bit_mask =  BIT64(PIN_BUTTON1), .mode = GPIO_MODE_INPUT, .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE };
const gpio_config_t gpioButton2 = { .pin_bit_mask =  BIT64(PIN_BUTTON2), .mode = GPIO_MODE_INPUT, .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE };
const gpio_config_t gpioButton3 = { .pin_bit_mask =  BIT64(PIN_BUTTON3), .mode = GPIO_MODE_INPUT, .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE };
const gpio_config_t gpioButton4 = { .pin_bit_mask =  BIT64(PIN_BUTTON4), .mode = GPIO_MODE_INPUT, .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE };
const gpio_config_t gpioCharging = { .pin_bit_mask =  BIT64(PIN_CHARGING), .mode = GPIO_MODE_INPUT, .pull_up_en = true, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE };
const gpio_config_t gpioUSBStatus = { .pin_bit_mask =  BIT64(PIN_USBSTATUS), .mode = GPIO_MODE_INPUT, .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE };

// Exchange structure
typedef struct __attribute__((packed))
{
  uint8_t data;
} exchange_struct_t;

// WS2812B LED
static led_strip_t *pStrip_a;
static uint16_t hue = 0;

// Event handle and ESPNow exchange data
static EventGroupHandle_t s_evt_group;
static exchange_struct_t exchangeData = { .data = 0 };

// ESPNow destination peer
const esp_now_peer_info_t espNowDestination =
{
  .peer_addr = MY_RECEIVER_MAC,
  .channel = 1,
  .ifidx = ESP_IF_WIFI_STA
};

// ADC settings
static adc_oneshot_unit_handle_t adc1_handle;
const adc_oneshot_unit_init_cfg_t adcInitConfig = { .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE, .clk_src = ADC_RTC_CLK_SRC_DEFAULT };
const adc_oneshot_chan_cfg_t adcChannelConfig = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_13 };

// Battery voltage
static int batteryVoltage_mV = 0;

// Charging state
static bool isCharging = 0;

//===============================================================
// Reads the button states
//===============================================================
static void ReadButtonStates()
{
  // Get button input states as fast as possible
  bitWrite(exchangeData.data, 0,  gpio_get_level(PIN_BUTTON1));
  bitWrite(exchangeData.data, 1,  gpio_get_level(PIN_BUTTON2));
  bitWrite(exchangeData.data, 2,  gpio_get_level(PIN_BUTTON3));
  bitWrite(exchangeData.data, 3,  gpio_get_level(PIN_BUTTON4));
}

//===============================================================
// Measures the battery voltage
//===============================================================
static void ReadBatteryVoltage(uint8_t count)
{
  // Get raw battery adc value
  int rawValue;
  int64_t rawValue_sum = 0;
  for (int index = 0; index < count; index++)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_VBATMESS, &rawValue));
    rawValue_sum += (int64_t)rawValue;
  }
  rawValue_sum /= count;

  // Calculate battery voltage
  batteryVoltage_mV = rawValue_sum * 5000 / 8191; // ADC max is 8191 digit (2500mV at ESP32S2), which corresponds to a battery voltage of 5000mV

  // Set battery voltage in exchange datas upper 4 bits
  uint8_t batteryData = rawValue_sum * 16 / 8191; // Battery voltage of 5000mV to 4 bit (16 digit)
  exchangeData.data &= ~0xF0;
  exchangeData.data |= (batteryData << 4) & 0xF0;
}

//===============================================================
// Reads the charging state
//===============================================================
static void ReadCharging()
{
  // Check for charging (Low active)
  isCharging = !gpio_get_level(PIN_CHARGING);
}

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
    ESP_ERROR_CHECK(nvs_flash_erase());
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
  ESP_ERROR_CHECK(esp_now_add_peer(&espNowDestination));
}

//===============================================================
// Sends an ESPNow frame
//===============================================================
static esp_err_t send_espnow_data(void)
{
  const uint8_t destination_mac[] = MY_RECEIVER_MAC;

  // Reset Feedback event
  s_evt_group = xEventGroupCreate();
  assert(s_evt_group);

  // Send ESPNow frame
  ESP_LOGI(TAG, "Sending frame");
  esp_err_t err = esp_now_send(destination_mac, (uint8_t*)&exchangeData, sizeof(exchangeData));
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

  ESP_LOGI(TAG, "Sent!");
  return ESP_OK;
}

//===============================================================
// Checks every possible next sender state
//===============================================================
static SenderState CheckNewSenderState(bool active, SenderState defaultState)
{
  if (active)
  {
    // Read button input states
    ReadButtonStates();

    // Read battery voltage
    ReadBatteryVoltage(100);

    // Read charging state
    ReadCharging();
  }

  // Check if any button is pressed
  if ((exchangeData.data & 0x0F) > 0)
  {
    // Transmit new frame
    return eTransmitting;
  }

  // Check if battery is charging
  if (isCharging)
  {
    // Show charging
    return eCharging;
  }

  // Otherwise stay in default state
  return defaultState;
}

//===============================================================
// Shown charging state while usb is connected
//===============================================================
static SenderState FctCharging()
{
  uint32_t red = 0;
  uint32_t green = 0;
  uint32_t blue = 0;

  // Charging active -> Run RGB colors
  hue += 3;
  hue %= 360;
  led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
  pStrip_a->set_pixel(pStrip_a, 0, red, green, blue);
  pStrip_a->refresh(pStrip_a, 100);

  // Wait 25 ms
  vTaskDelay(25 / portTICK_PERIOD_MS);

  // Determine new sender state
  return CheckNewSenderState(true, eShutOff);
}

//===============================================================
// Shown charging state while usb is connected
//===============================================================
static SenderState FctTransmit(void)
{
  // Send button and voltage data
  ESP_LOGI(TAG, "Send Frame");

  // Show BLUE color while sending frame
  pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 50);
  pStrip_a->refresh(pStrip_a, 100);

  // Send ESPNow frame
  esp_err_t error = send_espnow_data();

  if (error != ESP_OK)
  {
    // Show RED on error
    pStrip_a->set_pixel(pStrip_a, 0, 50, 0, 0);
    pStrip_a->refresh(pStrip_a, 100);
  }

  // Wait for next frame (First half of the time)
  vTaskDelay(RETRANSMISSION_TIME_MS / 2 / portTICK_PERIOD_MS);
  
  // Clear RGB LED
  pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 0);
  pStrip_a->refresh(pStrip_a, 100);
  
  // Wait for next frame (Second half of the time)
  vTaskDelay(RETRANSMISSION_TIME_MS / 2 / portTICK_PERIOD_MS);

  // Determine new sender state
  return CheckNewSenderState(true, eShutOff);
}

//===============================================================
// Shut power off
//===============================================================
static SenderState FctShutOff(void)
{
  // Shut power off
  ESP_LOGI(TAG, "Shut power off");
  gpio_set_level(PIN_KEEPALIVE, false);

  // Wait 100 ms
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Only comes up to here when the USB is plugged in
  // ...

  // Charging finished -> Show green
  pStrip_a->set_pixel(pStrip_a, 0, 0, 255, 0);
  pStrip_a->refresh(pStrip_a, 100);

  // Change state to wait for power off (Idle)
  return eIdle;
}

//===============================================================
// Wait in idle
//===============================================================
static SenderState FctIdle(void)
{
  // Wait 100ms
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Determine new sender state
  return CheckNewSenderState(true, eIdle);
}

//===============================================================
// Main task
//===============================================================
int app_main()
{
  // First of all set keep alive latch pin (Without ESP_ERROR_CHECK !!)
  gpio_config(&gpioKeepAlive);
  gpio_set_level(PIN_KEEPALIVE, true);

  // Initialize ESP Remote button inputs (Without ESP_ERROR_CHECK !!)
  gpio_config(&gpioButton1);
  gpio_config(&gpioButton2);
  gpio_config(&gpioButton3);
  gpio_config(&gpioButton4);

  // Get button input states as fast as possible
  ReadButtonStates();

  // Initialize battery measurement
  ESP_LOGI(TAG, "Initialize battery measurement");
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&adcInitConfig, &adc1_handle));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_VBATMESS, &adcChannelConfig));

  // Read battery voltage
  ReadBatteryVoltage(100);

  // Initialize RGB LED
  ESP_LOGI(TAG, "Initialize RGB LED");
  pStrip_a = led_strip_init(LED_RMT_CHANNEL, PIN_RGB_LED, NUM_LEDS);
  pStrip_a->clear(pStrip_a, 50);
  
  // Initialize charging state input
  ESP_LOGI(TAG, "Initialize charging state input");
  ESP_ERROR_CHECK(gpio_config(&gpioCharging));

  // Read charging state
  ReadCharging();

  // Initial debug values output
  ESP_LOGI(TAG, "Buttons state read: %u", exchangeData.data & 0x0F);
  ESP_LOGI(TAG, "Battery voltage is: %d mV", batteryVoltage_mV);
  ESP_LOGI(TAG, "Charging state is: %s", isCharging ? "true" : " false");

  // Initialize ESPNow
  ESP_LOGI(TAG, "Initialize ESPNow");
  init_espnow_slave();

  // Set initial sender state
  SenderState senderState = CheckNewSenderState(false, eShutOff);

  // Main loop
  while (true)
  {
    switch (senderState)
    {
      case eCharging:
        // Show charging
        senderState = FctCharging();
        break;
      case eTransmitting:
        // Send frame
        senderState = FctTransmit();
        break;
      case eIdle:
        // Wait in idle
        senderState = FctIdle();
        break;
      default:
      case eShutOff:
        // Shut off
        senderState = FctShutOff();
        break;
    }
  }
}