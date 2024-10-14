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
#define PIN_LED         15     // GPIO 15  -> Wemos S2 Mini PCB LED

//===============================================================
// Global Variables
//===============================================================
// Structure example to receive data
// Must match the sender structure
typedef struct exchange_struct_t
{
  uint8_t data;
} exchange_struct_t;

// Create a struct_message called exchangeData
exchange_struct_t exchangeData;

// LED brightness and speed
int16_t brightness = 127;
uint32_t lastToggle_ms = 0;
int32_t toggleSpeed_ms = 300;
bool toggle = false;

//===============================================================
// Callback function that will be executed when data is received
//===============================================================
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len)
{
  // Get MAC string
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  // Copy data
  memcpy(&exchangeData, incomingData, sizeof(exchangeData));

  // Debug output
  Serial.print("Bytes received: ");
  Serial.print(len);
  Serial.print(" -> data: ");
  Serial.print(exchangeData.data);
  Serial.print(" (MAC:");
  Serial.print(macStr);
  Serial.println(")");

  // Read single button values
  uint8_t button1 = bitRead(exchangeData.data, 0);
  uint8_t button2 = bitRead(exchangeData.data, 1);
  uint8_t button3 = bitRead(exchangeData.data, 2);
  uint8_t button4 = bitRead(exchangeData.data, 3);

  // Set Demo RGB LEDs
  brightness -=  button1 * 85;
  brightness +=  button2 * 85;
  brightness = max(brightness, (int16_t)10);
  brightness = min(brightness, (int16_t)255);
  toggleSpeed_ms -=  button4 * 200;
  toggleSpeed_ms +=  button3 * 200;
  toggleSpeed_ms = max(toggleSpeed_ms, (int32_t)100);
  toggleSpeed_ms = min(toggleSpeed_ms, (int32_t)500);
  analogWrite(PIN_LED, brightness);
}

//===============================================================
// Setup
//===============================================================
void setup()
{
  // Initialize serial monitor
  Serial.begin(115200);
  
  // Enable Status LED output
  pinMode(PIN_LED, OUTPUT);
  analogWrite(PIN_LED, brightness);
  
  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Init ESPNow
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESPNow");
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
      analogWrite(PIN_LED, brightness);
    }
    else
    {
      analogWrite(PIN_LED, 0);
    }
    toggle = !toggle;
    lastToggle_ms = millis();
  }
}
