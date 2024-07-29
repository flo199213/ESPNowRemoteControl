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
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

//===============================================================
// Defines
//===============================================================
#define PIN_LED     15     // GPIO 15  -> Wemos S2 Mini PCB LED

//===============================================================
// Global Variables
//===============================================================
// Structure example to receive data
// Must match the sender structure
typedef struct exchange_struct
{
  int throttle;
  int steering;
} exchange_struct;

// Create a struct_message called exchangeData
exchange_struct exchangeData;

//===============================================================
// Callback function that will be executed when data is received
//===============================================================
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&exchangeData, incomingData, sizeof(exchangeData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("throttle: ");
  Serial.println(exchangeData.throttle);
  Serial.print("steering: ");
  Serial.println(exchangeData.steering);
  Serial.println();

  // Toggle LED
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

//===============================================================
// Setup
//===============================================================
void setup()
{
  // Initialize serial monitor
  Serial.begin(115200);
  
  // Enable LED output
  pinMode(PIN_LED, OUTPUT);
  
  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP32 to full TX power and long range
  esp_wifi_set_max_tx_power(127);
  esp_wifi_set_protocol(WIFI_IF_STA , WIFI_PROTOCOL_LR);
  
  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

//===============================================================
// Main loop
//===============================================================
void loop()
{
}
