#include <Arduino.h>

#include "Arduino_JSON.h"
#include "HTTPClient.h"
#include "main_draw.h"
#include "main_erase.h"

String name = "Claude-Monetch";

unsigned long start_time = 0;

String server_address = "http://192.168.10.15";
int server_port = 5010;

HTTPClient http;

enum COMMAND
{
  WAIT,
  ERASE,
  DRAW
};

// Create pointer to function to call in loop, might be draw or erase
bool (*loop_function)() = NULL;

COMMAND command = WAIT;
void setup()
{
  // Configure onboard button as input
  pinMode(0, INPUT_PULLUP);

  Serial.begin(5000000);

  erase_pre_setup();
  draw_pre_setup();

  int return_code = 0;
  String address = server_address + ":" + server_port + "/connect";
  String body = "{\"name\": \"" + name + "\", \"ip\": \" " +
                WiFi.localIP().toString() + "\"}";

  Serial.println("Connecting to server");
  while (return_code != 200)
  {
    http.begin(address.c_str());
    http.addHeader("Content-Type", "application/json");
    return_code = http.POST(body);
    http.end();

    Serial.println("Server response: " + String(return_code));
  }

  // Call GET on /command to decide if we should erase or draw
  //   Query parameters: name

  while (command == WAIT)
  {
    address = server_address + ":" + server_port + "/command" + "?name=" + name;
    http.begin(address.c_str());
    http.addHeader("Content-Type", "application/json");
    //   call GET and retrieve JSON response
    return_code = http.GET();

    Serial.println("/command server response: " + String(return_code));

    if (return_code == 200)
    {
      // Parse JSON response
      // JSON expected:
      // {
      //   "command": "erase",
      // }

      JSONVar response = JSON.parse(http.getString());
      String command_str = (const char*)response["command"];

      Serial.println("Command: " + command_str);
      if (command_str == "erase")
      {
        command = ERASE;
      }
      else if (command_str == "draw")
      {
        command = DRAW;
      }
      else
      {
        command = WAIT;
      }
    }

    http.end();

    delay(1000);
  }

  start_time = micros();
  if (command == ERASE)
  {
    erase_setup();
    loop_function = erase_loop;
  }
  else if (command == DRAW)
  {
    draw_setup();
    loop_function = draw_loop;
  }
}

void loop()
{
  if (loop_function())
  {
    Serial.println("Complete");
    unsigned long completion_time = micros() - start_time;
    float completion_time_seconds = completion_time / 1000000.0;

    // We are complete now, send a complete message to the server
    String address = server_address + ":" + server_port;
    String body;

    if (command == DRAW)
    {
      address += "/drawing_complete";

      body = "{\"name\": \"" + name +
             "\", \"drawing_time\": " + String(completion_time_seconds) + "}";
    }
    else if (command == ERASE)
    {
      address += "/erasing_complete";

      body = "{\"name\": \"" + name +
             "\", \"erasing_time\": " + String(completion_time_seconds) + "}";
    }

    int return_code = 0;
    while (return_code != 200)
    {
      http.begin(address.c_str());
      http.addHeader("Content-Type", "application/json");
      return_code = http.POST(body);
      http.end();

      Serial.println("Server response: " + String(return_code));
    }

    // Reset the ESP32
    esp_restart();
  }
}
