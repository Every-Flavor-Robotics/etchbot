#include <WebSocketsServer.h>  // Add this for WebSockets
#include <WiFi.h>

#include "Arduino.h"
#include "Arduino_JSON.h"
#include "configurable.h"
#include "pid_manager.h"

// Type for saving and loading calibration parameters to/from EEPROM
const int DATA_PACKET_LEN =
    sizeof(long) + 10 * sizeof(char) + sizeof(float) + sizeof(int);
typedef union
{
  struct __attribute__((packed))
  {
    char name[10];
    long timestamp;
    float sampleValue;
  };

  uint8_t raw[DATA_PACKET_LEN];
} data_packet_t;

MotorGo::PIDManager pid_manager;
// declare and configure custom velocity controller object
MotorGo::PIDParameters velocity_controller_params;

WebSocketsServer webSocket =
    WebSocketsServer(1234);  // Use a standard WebSocket port (e.g., 8080)

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length)
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
    {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
                    ip[1], ip[2], ip[3], payload);
    }
    break;
    case WStype_TEXT:
      // Handle incoming text messages (you likely won't need this for your data
      // streaming)
      break;
    case WStype_BIN:
      // We'll assume you send your data in binary format
      //   handleBinaryData(num, payload, length);
      break;
  }
}

// void handleBinaryData(uint8_t num, uint8_t *payload, size_t length)
// {
//   // You might need to deserialize your DataPacket here if necessary

//   // Broadcast the data to all connected clients
//   webSocket.broadcastBIN(payload, length);
// }

// Helper function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = micros();

  if (now - last_print_time > 1e6 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

void setup()
{
  Serial.begin(115200);

  pid_manager.add_controller("/velocity", velocity_controller_params,
                             []() { ; });

  Serial.println("WiFi connected!");
  //   Print the IP address
  Serial.println(WiFi.localIP());
  // initialize the PID manager
  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  // Start the WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

float i = 0;
unsigned long last_print_time = 0;
void loop()
{
  long currentTime = micros();
  i += 0.01;

  unsigned long now = micros();

  if (now - last_print_time > 1e6 / 20)
  {
    data_packet_t dataPacket;
    dataPacket.timestamp = currentTime;
    dataPacket.sampleValue = sin(i / 20);
    strcpy(dataPacket.name, "sin");

    webSocket.broadcastBIN(dataPacket.raw, DATA_PACKET_LEN);

    data_packet_t dataPacket2;
    dataPacket2.timestamp = currentTime;
    dataPacket2.sampleValue = cos(i / 20);
    strcpy(dataPacket2.name, "cos");

    webSocket.broadcastBIN(dataPacket2.raw, DATA_PACKET_LEN);
    last_print_time = now;
  }

  // Broadcast the data to all connected clients
  webSocket.loop();  // Handle WebSocket events
}