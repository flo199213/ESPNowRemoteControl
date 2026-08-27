/**
 * ESP32 Get MAC
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

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_mac.h> // Definition of esp_read_mac and esp_efuse_mac_get_default


//===============================================================
// Constants
//===============================================================
static const char* TAG = "main";

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
void PrintMacAddress()
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
// Setup function
//===============================================================
void setup()
{
  Serial.begin(115200);
}
 
//===============================================================
// Loop function
//===============================================================
void loop()
{
  if (Serial.available() > 0)
  {
    Serial.readString();
    PrintMacAddress();
  }
}

