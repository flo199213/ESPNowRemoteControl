/**
 * ESP32 ESPNow Receiver
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
 * If the Wemos S2 Mini is programmed via the Arduino IDE, the
 * "USB CDC On Boot" flag must be set at all times. This flag
 * causes the Wemos S2 Mini to report as a COM interface immediately
 * after booting via USB. This means that the microcontroller can
 * be programmed via the Arduino Ide WITHOUT having to press the 
 * "BOOT" and "RESET" buttons again.
 * 
 * ==============================================================
 */

//===============================================================
// Includes
//===============================================================
#include <esp_log.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <WiFi.h>

//===============================================================
// Defines
//===============================================================
#define PIN_LED         15     // GPIO 15  -> Wemos S2 Mini PCB LED

//===============================================================
// Constants
//===============================================================
static const char* TAG = "main";
//static const uint8_t allowedSenderMAC[] = { 0x80, 0x65, 0x99, 0xfb, 0x43, 0x2A };

//===============================================================
// Global Variables
//===============================================================
// Structure example to receive data - PACKED avoids Memory Errors
// Must match the sender structure
typedef struct __attribute__((packed)) exchange_struct_t
{
  uint8_t data;
} exchange_struct_t;

// Create a struct_message called messageData
exchange_struct_t messageData;

// Debug LED speed
uint32_t lastToggle_ms = 0;
const int32_t toggleSpeed_ms = 800;
bool toggle = false;

// Flash counter
volatile uint8_t flashCount = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//===============================================================
// Helper function to print the MAC address in a formatted style
// to the Serial Monitor
//===============================================================
void PrintMacAddress(const char* label, esp_err_t espError, uint8_t* mac)
{
  if (espError == ESP_OK)
  {
    ESP_LOGI(TAG, "%-30s: %02X:%02X:%02X:%02X:%02X:%02X", label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  else
  {
    ESP_LOGE(TAG, "%-30s: Failed", label);
  }
}

//===============================================================
// Helper function to print the MAC address in a formatted style
// to the Serial Monitor
//===============================================================
void PrintAllMacAddresses()
{
  uint8_t mac[6];
  ESP_LOGI(TAG, "==================================================");
  ESP_LOGI(TAG, "READING ESP Board MAC ADDRESSES");
  ESP_LOGI(TAG, "==================================================");
  PrintMacAddress("Factory Base eFuse MAC", esp_efuse_mac_get_default(mac), mac);               // Factory-burned base MAC address from eFuses
  ESP_LOGI(TAG, "--------------------------------------------------");
  PrintMacAddress("Active Base MAC (RAM)", esp_read_mac(mac, ESP_MAC_BASE), mac);               // Currently active base MAC (might be modified via software in RAM)
  PrintMacAddress("Wi-Fi Station (STA) MAC", esp_read_mac(mac, ESP_MAC_WIFI_STA), mac);         // Wi-Fi Station (STA) MAC address
  PrintMacAddress("Wi-Fi Access Point (AP) MAC", esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP), mac);  // Wi-Fi Access Point (SoftAP) MAC address
  PrintMacAddress("Bluetooth (BT) MAC", esp_read_mac(mac, ESP_MAC_BT), mac);                    // Bluetooth MAC address (ESP32-S2 does not support Bluetooth)
  PrintMacAddress("Ethernet (ETH) MAC", esp_read_mac(mac, ESP_MAC_ETH), mac);                   // Ethernet MAC address
  ESP_LOGI(TAG, "==================================================");
}

//===============================================================
// Callback function that will be executed when data is received
//===============================================================
void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t* data, int length)
{  
  // Check for valid sender info
  if (esp_now_info == NULL)
  {
    ESP_LOGE(TAG, "Error: esp_now_info is null");
    return;
  }
  uint8_t* mac = esp_now_info->src_addr;

  // Print received bytes
  ESP_LOGI(TAG, "Bytes received: %d - MAC: %02x:%02x:%02x:%02x:%02x:%02x", length, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Check for valid sender MAC
  /*if (memcmp(mac, allowedSenderMAC, 6) != 0)
  {
    ESP_LOGE(TAG, " -> Invalid Sender Error");
    return;
  }*/

  // Check for valid data length
  if (length != sizeof(messageData))
  {
    ESP_LOGE(TAG, " -> Invalid Data Length");
    return;
  }
  
  // Copy data
  memcpy(&messageData, data, sizeof(messageData));

  // Read single button values
  uint8_t button1 = bitRead(messageData.data, 0);
  uint8_t button2 = bitRead(messageData.data, 1);
  uint8_t button3 = bitRead(messageData.data, 2);
  uint8_t button4 = bitRead(messageData.data, 3);

  // Calculate battery voltage
  uint32_t batteryData = (messageData.data >> 4) & 0x0F;
  double batteryVoltage_V = (double)batteryData * 5.0 / 16.0; // Battery voltage of 4 bit (0-16) to 0-5000mV

  // Print Data
  ESP_LOGI(TAG, " - Buttons: B1=%d, B2=%d, B3=%d, B4=%d, BATT=%fV", button1, button2, button3, button4, batteryVoltage_V);

  // Enter mux to avoid race condition
  portENTER_CRITICAL_ISR(&mux);

  // Set flashCount value
  flashCount = 0;
  if (button1)
  {
    flashCount = 1;
  }
  else if (button2)
  {
    flashCount = 2;
  }
  else if (button3)
  {
    flashCount = 3;
  }
  else if (button4)
  {
    flashCount = 4;
  }

  // Exit mux to avoid race condition
  portEXIT_CRITICAL_ISR(&mux);
}

//===============================================================
// Setup
//===============================================================
void setup()
{
  // Start USB CDC // (redundant, for build verification purposes only)
  USBSerial.begin(); // <--- If you get an compile error here, you must
  // enable "USB CDC On Boot" in the Arduino IDE Target settings for ESP32-S2!

  // Initialize serial monitor
  Serial.begin(115200);
  
  // Route ESP-IDF log messages (like ESP_LOGI) to the USB CDC interface
  Serial.setDebugOutput(true);
  
  // Give the USB Serial port a brief moment to connect
  delay(1000);
  ESP_LOGI(TAG, "ESP32Sx ESPNow Receiver");
  
  // Print all mac adresses from ESP module
  PrintAllMacAddresses();

  // Enable Status LED output
  pinMode(PIN_LED, OUTPUT);
  analogWrite(PIN_LED, 100);
  
  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Init ESPNow
  if (esp_now_init() != ESP_OK)
  {
    ESP_LOGE(TAG, "Error initializing ESPNow");
    return;
  }

  // Set ESP32 to long range and full TX power
  esp_wifi_set_protocol(WIFI_IF_STA , WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(84); // 21dBm (Max TX power)
  
  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

//===============================================================
// Main loop
//===============================================================
void loop()
{
  if (millis() - lastToggle_ms > toggleSpeed_ms)
  {
    if (toggle)
    {
      analogWrite(PIN_LED, 5); // Blink super dark (esp-now frame will flash super bright)
    }
    else
    {
      analogWrite(PIN_LED, 0);
    }
    toggle = !toggle;
    lastToggle_ms = millis();
  }

  uint8_t currentFlashCount = 0;

  // Enter mux to avoid race condition
  portENTER_CRITICAL_ISR(&mux);

  currentFlashCount = flashCount;
  flashCount = 0;

  // Exit mux to avoid race condition
  portEXIT_CRITICAL_ISR(&mux);

  if (currentFlashCount > 0)
  {
    for (uint8_t index = 0; index < currentFlashCount; index++)
    {
      analogWrite(PIN_LED, 255); // Flash super bright
      delay(200);
      analogWrite(PIN_LED, 0);
      delay(300);
    }
  }
}
