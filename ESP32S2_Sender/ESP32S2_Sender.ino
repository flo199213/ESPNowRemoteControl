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
#define PIN_BUTTON  0     // GPIO 0  -> Wemos S2 Mini PCB push button

//===============================================================
// Global Variables
//===============================================================
// ESP Receiver's MAC address
uint8_t broadcastAddress[] = { 0x84, 0xfc, 0xe6, 0xd1, 0x21, 0x94 };

// Structure example to send data
// Must match the sender structure
typedef struct exchange_struct
{
  int throttle;
  int steering;
} exchange_struct;

// Create a struct_message called exchangeData
exchange_struct exchangeData;

// ESP Now pairing info
esp_now_peer_info_t peerInfo;

// Debounce variable
bool buttonDebounce = true;

//===============================================================
// Callback when data is sent
//===============================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  Serial.print("Packet to: ");

  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//===============================================================
// Setup
//===============================================================
void setup()
{
  // Initialize serial monitor
  Serial.begin(115200);

  // Enable button input
  pinMode(PIN_BUTTON, INPUT_PULLUP);
 
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
  
  // Once ESPNow is successfully Init, we will register for send CB to get send info
  esp_now_register_send_cb(OnDataSent);
   
  // Set peer (Pairing info)
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  
  // Register peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

//===============================================================
// Main loop
//===============================================================
void loop()
{
  // Check for button press
  if (!digitalRead(PIN_BUTTON) &&
    buttonDebounce)
  {
    buttonDebounce = false;

    // Set random data
    exchangeData.throttle = random(0, 20);
    exchangeData.steering = random(0, 20);
  
    // Send esp now frame
    esp_err_t result = esp_now_send(0, (uint8_t *) &exchangeData, sizeof(exchange_struct));
    Serial.println(result == ESP_OK ? "Sent with success" : "Error sending the data");
    
    delay(500);
  }
  else
  {
    buttonDebounce = true;
  }
}


