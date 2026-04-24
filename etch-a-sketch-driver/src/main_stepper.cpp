#include <Arduino.h>

#include "main_stepper_draw.h"
#include "main_stepper_erase.h"

String robot_name = "Etchbot-Stepper";

unsigned long start_time = 0;

enum COMMAND
{
    WAIT,
    ERASE,
    DRAW
};

// Create pointer to function to call in loop, might be draw or erase
bool (*loop_function)() = NULL;

COMMAND command = WAIT;

// Read a line from Serial, blocking until newline received
String serial_read_line()
{
    String line = "";
    while (true)
    {
        if (Serial.available() > 0)
        {
            char c = Serial.read();
            if (c == '\n')
            {
                line.trim();
                return line;
            }
            if (c != '\r')
            {
                line += c;
            }
        }
        else
        {
            delay(10);
        }
    }
}

void setup()
{
    // Configure onboard button as input
    pinMode(0, INPUT_PULLUP);

    Serial.begin(115200);
    while (!Serial)
    {
        delay(50);
    }

    // Small delay to let USB CDC settle
    delay(1000);

    erase_pre_setup();
    draw_pre_setup();

    // Send HELLO and wait for COMMAND response
    Serial.println("Connecting to server via Serial...");

    while (command == WAIT)
    {
        Serial.println("HELLO:" + robot_name);

        // Wait for response with timeout
        unsigned long send_time = millis();
        while (!Serial.available())
        {
            if (millis() - send_time > 3000)
            {
                break;  // Timeout, will resend HELLO
            }
            delay(10);
        }

        if (Serial.available())
        {
            String response = serial_read_line();
            Serial.println("Received: " + response);

            if (response.startsWith("COMMAND:"))
            {
                String command_str = response.substring(8);
                command_str.trim();

                if (command_str == "draw")
                {
                    command = DRAW;
                }
                else if (command_str == "erase")
                {
                    command = ERASE;
                }
                // else stays WAIT, will retry
            }
        }

        if (command == WAIT)
        {
            delay(1000);
        }
    }

    Serial.println("Command received: " + String(command == DRAW ? "draw" : "erase"));

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

        // Send completion message over serial
        if (command == DRAW)
        {
            Serial.println("DONE:draw:" + String(completion_time_seconds));
        }
        else if (command == ERASE)
        {
            Serial.println("DONE:erase:" + String(completion_time_seconds));
        }

        // Wait a moment for the message to be sent, then restart
        delay(1000);
        esp_restart();
    }
}
