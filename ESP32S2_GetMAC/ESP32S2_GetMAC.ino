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

void setup()
{
  Serial.begin(115200);  
}
 
void loop()
{
  if (Serial.available() > 0)
  {
    Serial.read();
    Serial.print("ESP Board MAC Address: ");

    WiFi.mode(WIFI_STA);
    WiFi.begin();
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);

    if (ret == ESP_OK)
    {
      Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    }
    else
    {
      Serial.println("Failed to read MAC address");
    }
  }
}

