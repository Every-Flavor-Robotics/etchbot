#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>  // Ensure this library is included
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include <atomic>

#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "pid_manager.h"

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

#define FLIPPING_VELOCITY_RAD_PER_SEC 1.2

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& tilt_motor = motorgo_mini.ch0;

MotorGo::MotorConfiguration gm3506_config;
MotorGo::ChannelConfiguration tilt_motor_config;

// Velocity controller parameters
MotorGo::PIDParameters erasing_velocity_controller;

// Position controller parameters
MotorGo::PIDParameters erasing_position_controller_parameters;
MotorGo::PIDParameters holding_position_controller_parameters;
PIDController position_controller(0.0, 0.0, 0.0, 0.0, 0.0);

// Configure WiFi communications
bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

unsigned long last_time = micros();
unsigned long hold_time = 4 * 1e6;

std::atomic<float> target_pos{0.0};
std::atomic<float> erase_begin{false};
std::atomic<float> disable_flag{false};
std::atomic<float> enable_flag{false};

int cur_cycle_count = 0;
int erase_cycles = 3;

bool current_pos = true;
int erase_motor_pwm = 0.0;

bool zeroing = false;
unsigned long zero_start_time = 0;
unsigned long zero_hold_time = 4 * 1e6;

float error_tolerance = 0.1;

void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

/// @brief  Return angle in range [0, 2pi)
/// @param angle Angle in radians
/// @return Angle in range [0, 2pi)
float get_angle_absolute(float angle)
{
  // Return an angle between 0 and 2pi
  while (angle < 0)
  {
    angle += 2 * PI;
  }
  //   Not inclusive of 2pi
  while (angle >= 2 * PI)
  {
    angle -= 2 * PI;
  }
  return angle;
}

/// @brief  Get a target angle that is in the correct direction from the current
/// @param cur_angle Current angle
/// @param target_angle Target angle in range [0, 2pi)
/// @param direction Direction to move in
/// @return Target angle
float get_target(float cur_angle, float target_angle, int direction)
{
  if (direction == 1)
  {
    // Work in the negative direction, until we're less than the target
    // This ensures that we will never be more than one rotation away
    while (target_angle > cur_angle)
    {
      target_angle -= 2 * PI;
    }
    // Go forward until we're greater than the target
    while (target_angle < cur_angle)
    {
      target_angle += 2 * PI;
    }
  }
  else
  {
    while (target_angle < cur_angle)
    {
      target_angle += 2 * PI;
    }

    while (target_angle > cur_angle)
    {
      target_angle -= 2 * PI;
    }
  }
  return target_angle;
}

float zero_position = get_angle_absolute(PI / 2.0 + PI / 6.0 + 0.2);
// float erase_position_1 = get_angle_absolute(11 * PI / 12.0 + zero_position);
// float erase_position_2 = get_angle_absolute(PI / 12.0 + zero_position);
float erase_position_1 = get_angle_absolute(11.0 * PI / 12.0 + zero_position);
float erase_position_2 = get_angle_absolute(4.0 * PI / 12.0 + zero_position);

// blue -> 41
// yellow -> 42
uint8_t input_pin = 41;
uint8_t output_pin = 42;

void setup()
{
  Serial.begin(5000000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Try connecting 5 times, then give up
  int connect_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && connect_attempts < 5)
  {
    delay(1000);
    Serial.print(".");
    connect_attempts++;
  }

  // Initialize mDNS with the desired hostname
  if (!MDNS.begin("claude-monetch-eraser"))
  {
    Serial.println("Error starting mDNS responder!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  ArduinoOTA
      .onStart(
          []()
          {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
              type = "sketch";
            else  // U_SPIFFS
              type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount
            // SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
          })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress(
          [](unsigned int progress, unsigned int total)
          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError(
          [](ota_error_t error)
          {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
              Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
              Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
              Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
              Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)
              Serial.println("End Failed");
          });

  // Set the OTA hostname to match the mDNS hostname
  ArduinoOTA.setHostname("claude-monetch-eraser");

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(input_pin, INPUT);
  pinMode(output_pin, OUTPUT);

  digitalWrite(42, LOW);

  gm3506_config.pole_pairs = 11;
  gm3506_config.phase_resistance = 5.6;
  gm3506_config.calibration_voltage = 2.0;

  tilt_motor_config.motor_config = gm3506_config;
  tilt_motor_config.power_supply_voltage = 9.0;
  tilt_motor_config.reversed = true;

  tilt_motor.init(tilt_motor_config, false);

  erasing_velocity_controller.p = 1.0;
  erasing_velocity_controller.lpf_time_constant = 0.1;

  holding_position_controller_parameters.p = 9;
  holding_position_controller_parameters.i = 0.1;
  holding_position_controller_parameters.d = 0.9;
  holding_position_controller_parameters.lpf_time_constant = 0.09;

  // Configure the position controller
  erasing_position_controller_parameters.p = 3.5;
  erasing_position_controller_parameters.d = 0.5;

  // Set parameters for holding mode
  position_controller.P = holding_position_controller_parameters.p;
  position_controller.I = holding_position_controller_parameters.i;
  position_controller.D = holding_position_controller_parameters.d;
  position_controller.output_ramp =
      holding_position_controller_parameters.output_ramp;
  position_controller.limit = holding_position_controller_parameters.limit;

  // Configure the velocity controller used for flipping
  tilt_motor.set_velocity_controller(erasing_velocity_controller);
  tilt_motor.set_velocity_limit(FLIPPING_VELOCITY_RAD_PER_SEC);

  // Pass the velocity controller in, ONLY to update the internal low-pass
  tilt_motor.set_position_controller(holding_position_controller_parameters);
  // Set voltage control mode for holding
  tilt_motor.set_control_mode(MotorGo::ControlMode::Voltage);

  pinMode(CH1_UH, OUTPUT);
  pinMode(CH1_UL, OUTPUT);
  pinMode(CH1_VH, OUTPUT);
  pinMode(CH1_VL, OUTPUT);

  digitalWrite(CH1_UH, LOW);
  digitalWrite(CH1_UL, LOW);
  digitalWrite(CH1_VH, LOW);
  digitalWrite(CH1_VL, LOW);

  ledcSetup(6, 20000, 8);
  ledcSetup(7, 20000, 8);

  ledcAttachPin(CH1_UL, 6);
  ledcAttachPin(CH1_VL, 7);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  delay(2000);

  // Find the shortest distance to zero position, for holding
  float cur_position = tilt_motor.get_position();
  float target_forward = get_target(cur_position, zero_position, 1);
  float target_backward = get_target(cur_position, zero_position, -1);

  if (abs(target_forward - cur_position) < abs(target_backward - cur_position))
  {
    target_pos = target_forward;
  }
  else
  {
    target_pos = target_backward;
  }

  Serial.print("Cur: ");
  Serial.print(cur_position);
  Serial.print(" Forward: ");
  Serial.print(target_forward);
  Serial.print(" Backward: ");
  Serial.print(target_backward);
  Serial.print(" Target: ");
  Serial.println(target_pos);

  Serial.println(target_pos);
  enable_flag.store(true);

  delay(4000);
}

void loop_foc(void* pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    tilt_motor.loop();

    float output = position_controller(target_pos - tilt_motor.get_position());

    // Switch between velocity control (for erasing) and voltage control (for
    // holding)
    if (erase_begin.load())
    {
      tilt_motor.set_target_velocity(output);
    }
    else
    {
      tilt_motor.set_target_voltage(output);
    }

    if (disable_flag.load())
    {
      tilt_motor.disable();
      disable_flag.store(false);
    }
    if (enable_flag.load())
    {
      tilt_motor.enable();
      enable_flag.store(false);
    }

    esp_task_wdt_reset();
  }
}

void loop()
{
  ArduinoOTA.handle();
  // Update the motors
  float angle = tilt_motor.get_position();

  // Wait until we receive a high on the input pin
  if (digitalRead(input_pin) == HIGH && !erase_begin)
  {
    digitalWrite(output_pin, HIGH);

    disable_flag.store(true);
    delay(100);

    // Switch to erasing parameters
    position_controller.P = erasing_position_controller_parameters.p;
    position_controller.I = erasing_position_controller_parameters.i;
    position_controller.D = erasing_position_controller_parameters.d;
    position_controller.output_ramp =
        erasing_position_controller_parameters.output_ramp;
    position_controller.limit = erasing_position_controller_parameters.limit;

    position_controller.reset();

    // Switch to velocity control mode for erasing
    tilt_motor.set_control_mode(MotorGo::ControlMode::Velocity);

    enable_flag.store(true);

    erase_begin = true;
  }

  if (erase_begin)
  {
    erase_motor_pwm = 255 * (1.8 / tilt_motor_config.power_supply_voltage);

    digitalWrite(CH1_UH, HIGH);
    ledcWrite(7, erase_motor_pwm);

    if (cur_cycle_count >= erase_cycles)
    {
      if (!zeroing)
      {
        zeroing = true;
        zero_start_time = micros();
        target_pos = get_target(angle, zero_position, -1);
        position_controller.reset();
      }

      if (micros() - zero_start_time > zero_hold_time)
      {
        zeroing = false;
        erase_begin = false;
        cur_cycle_count = 0;

        Serial.println("Erasing complete");

        // Stop motors
        disable_flag.store(true);

        ledcWrite(7, 0);

        // Set output to low to indicate that we are done erasing
        digitalWrite(output_pin, LOW);

        delay(1000);

        // Restart the board
        ESP.restart();
      }
    }

    if (micros() - last_time > hold_time && !zeroing)
    {
      if (current_pos)
      {
        target_pos = get_target(angle, erase_position_1, 1);
        Serial.println("Erasing position 1");
        Serial.println("Current position: " + String(angle));
        Serial.println("Target position (raw): " + String(erase_position_1));
        Serial.println("Target position: " + String(target_pos));
        current_pos = false;
      }
      else
      {
        target_pos = get_target(angle, erase_position_2, -1);
        Serial.println("Erasing position 2");
        Serial.println("Current position: " + String(angle));
        Serial.println("Target position (raw): " + String(erase_position_2));
        Serial.println("Target position: " + String(target_pos));
        current_pos = true;
        cur_cycle_count++;

        Serial.println("Cycle count: " + String(cur_cycle_count));
      }

      last_time = micros();
    }
  }
  else
  {
    digitalWrite(CH1_UH, LOW);
    ledcWrite(7, 0);
  }
}