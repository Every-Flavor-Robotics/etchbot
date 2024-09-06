#include <Arduino.h>
#include <WebSocketsServer.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

uint8_t erase_out_pin = QWIIC_SDA;
uint8_t erase_in_pin = QWIIC_SCL;

void erase_pre_setup()
{
  pinMode(erase_out_pin, OUTPUT);
  pinMode(erase_in_pin, INPUT);

  digitalWrite(erase_out_pin, LOW);
}

//   Wait for erase button to be pressed
//   while (digitalRead(0) == HIGH)
//   {
//     delay(100);
//     Serial.println(digitalRead(erase_in_pin));
//   }

void erase_setup()
{
  digitalWrite(erase_out_pin, HIGH);
  delay(1000);

  while (digitalRead(erase_in_pin) == HIGH)
  {
    Serial.println(digitalRead(erase_in_pin));
    delay(100);
  }

  Serial.println("Erasing Complete");

  digitalWrite(erase_out_pin, LOW);
}

bool erase_loop() { return true; }
