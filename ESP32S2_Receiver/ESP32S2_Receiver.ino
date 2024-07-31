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
#include <Adafruit_NeoPixel.h>

//===============================================================
// Defines
//===============================================================
#define PIN_LED         15     // GPIO 15  -> Wemos S2 Mini PCB LED
#define PIN_RGB_LED     16     // GPIO 16  -> WS2812B Demo LED

#define RGB_LED_COUNT   1

//===============================================================
// Global Variables
//===============================================================
// Structure example to receive data
// Must match the sender structure
typedef struct exchange_struct
{
  int buttons;
} exchange_struct;

// Create a struct_message called exchangeData
exchange_struct exchangeData;

// Demo RGB LED
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(RGB_LED_COUNT, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);

//===============================================================
// Callback function that will be executed when data is received
//===============================================================
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&exchangeData, incomingData, sizeof(exchangeData));
  Serial.print("Bytes received: ");
  Serial.print(len);
  Serial.print(" -> buttons: ");
  Serial.println(exchangeData.buttons);

  // Read single button values
  uint8_t button1 = bitRead(exchangeData.buttons, 0);
  uint8_t button2 = bitRead(exchangeData.buttons, 1);
  uint8_t button3 = bitRead(exchangeData.buttons, 2);
  uint8_t button4 = bitRead(exchangeData.buttons, 3);

  // Toggle LED
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));

  // Set Demo RGB LED
  uint8_t red = (button1 ? 125 : 0);
  uint8_t green = button2 ? 125 : 0 + (button4 ? 125 : 0);
  uint8_t blue = (button3 ? 125 : 0) + (button4 ? 125 : 0);
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();
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
  
  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Init ESPNow
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESPNow");
    return;
  }

  // Set ESP32 to full TX power and long range
  esp_wifi_set_max_tx_power(127);
  esp_wifi_set_protocol(WIFI_IF_STA , WIFI_PROTOCOL_LR);
  
  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Initialize RGB LED
  pixels.begin();
  pixels.setBrightness(255);

  // Startup sequence
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
  pixels.show();
  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
  pixels.show();
  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue
  pixels.show();
  delay(1000);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // White
  pixels.show();
}

//===============================================================
// Main loop
//===============================================================
void loop()
{
}
