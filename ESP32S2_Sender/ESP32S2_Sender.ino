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
#define PIN_BUTTON1     3     // GPIO 3
#define PIN_BUTTON2     37    // GPIO 37
#define PIN_BUTTON3     11    // GPIO 11
#define PIN_BUTTON4     16    // GPIO 16
#define PIN_LED_GND     35    // GPIO 35
#define PIN_LED_VCC     5     // GPIO 5
#define PIN_KEEPALIVE   7     // GPIO 7

//===============================================================
// Global Variables
//===============================================================
// ESP Receiver's MAC address
uint8_t broadcastAddress[] = { 0x84, 0xfc, 0xe6, 0xd1, 0x21, 0x94 };

// Structure example to send data
// Must match the sender structure
typedef struct exchange_struct_t
{
  uint8_t buttons;
} exchange_struct_t;

// Create a struct_message called exchangeData
exchange_struct_t exchangeData;

// ESP Now pairing info
esp_now_peer_info_t peerInfo;

//===============================================================
// Setup
//===============================================================
void setup()
{
  // Initialize pin modes
  pinMode(PIN_BUTTON1, INPUT_PULLDOWN);
  pinMode(PIN_BUTTON2, INPUT_PULLDOWN);
  pinMode(PIN_BUTTON3, INPUT_PULLDOWN);
  pinMode(PIN_BUTTON4, INPUT_PULLDOWN);
  pinMode(PIN_KEEPALIVE, OUTPUT);
  pinMode(PIN_LED_GND, OUTPUT);
  pinMode(PIN_LED_VCC, OUTPUT);
  
  // Set keep alive
  digitalWrite(PIN_KEEPALIVE, HIGH);

  // Read button input
  ReadButtonsState();

  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Send frame with button state
  SendFrame();
}

//===============================================================
// Main loop
//===============================================================
void loop()
{
  // Wait 1 second
  delay(1000);

  // Read button input
  ReadButtonsState();
  
  // Resend frame with button state every Second
  SendFrame();
}

//===============================================================
// Reads the button states
//===============================================================
void ReadButtonsState()
{
  // Read button state
  exchangeData.buttons = digitalRead(PIN_BUTTON4) << 3 | digitalRead(PIN_BUTTON3) << 2 | digitalRead(PIN_BUTTON2) << 1 | digitalRead(PIN_BUTTON1);
}

//===============================================================
// Send a frame with button states
//===============================================================
void SendFrame()
{
  // Set keep alive for complete frame
  pinMode(PIN_KEEPALIVE, OUTPUT);
  digitalWrite(PIN_KEEPALIVE, HIGH);

  // Set LED on (pwm to 50%)
  analogWrite(PIN_LED_VCC, 125);

  // Init ESPNow
  if (esp_now_init() == ESP_OK)
  {
    // Set ESP32 to full TX power and long range
    esp_wifi_set_max_tx_power(84); // 21dBm (Max TX power)
    esp_wifi_set_protocol(WIFI_IF_STA , WIFI_PROTOCOL_LR);
        
    // Set peer (Pairing info)
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    
    // Register peer
    if (esp_now_add_peer(&peerInfo) == ESP_OK)
    {
      // Send esp now frame
      esp_now_send(0, (uint8_t *) &exchangeData, sizeof(exchange_struct_t));
    }
  }

  // Set LED off
  analogWrite(PIN_LED_VCC, 0);

  // Shut off
  digitalWrite(PIN_KEEPALIVE, LOW);
  pinMode(PIN_KEEPALIVE, INPUT);
}
