#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <motorgo_mini.h>

#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "pid_manager.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

#define FLIPPING_VELOCITY_RAD_PER_SEC 2.0

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& tilt_motor = motorgo_mini.ch0;

MotorGo::MotorConfiguration gm3506_config;
MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

// Velocity controller parameters
MotorGo::PIDParameters velocity_controller_parameters_ch0;
MotorGo::PIDParameters velocity_controller_parameters_ch1;

// Position controller parameters
MotorGo::PIDParameters erasing_position_controller_parameters;
MotorGo::PIDParameters holding_position_controller_parameters;
PIDController position_controller(0.0, 0.0, 0.0, 0.0, 0.0);

// configure wifi communications
bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

unsigned long last_time = micros();
unsigned long hold_time = 2.5 * 1e6;

float zero_position = 0.06;
float erase_position_1 = PI;
float erase_position_2 = PI / 3;
float target_pos = 0;

int cur_cycle_count = 0;
int erase_cycles = 5;

bool current_pos = true;
int erase_motor_pwm = 0.0;

bool zeroing = false;
unsigned long zero_start_time = 0;
unsigned long zero_hold_time = 8 * 1e6;

float error_tolerance = 0.1;

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    // motor_left.enable();
    tilt_motor.enable();
  }
  else
  {
    Serial.println("Disabling motors");
    // motor_left.disable();
    tilt_motor.disable();
  }
}

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
    while (target_angle < cur_angle)
    {
      target_angle += 2 * PI;
    }
  }
  else
  {
    while (target_angle > cur_angle)
    {
      target_angle -= 2 * PI;
    }
  }
  return target_angle;
}

// blue -> 41
// yellow -> 42
uint8_t input_pin = 41;
uint8_t output_pin = 42;
bool erase_begin = false;
void setup()
{
  Serial.begin(5000000);

  delay(5000);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("etchbot-1-eraser");
  WiFi.begin(ssid, password);

  //   Try connecting 5 times, then give up
  int connect_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && connect_attempts < 5)
  {
    delay(1000);
    Serial.print(".");
    connect_attempts++;
  }

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

  ArduinoOTA.setHostname("etchbot-1-eraser");

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

  config_ch1.motor_config = gm3506_config;
  config_ch1.power_supply_voltage = 9.0;
  config_ch1.reversed = true;

  tilt_motor.init(config_ch1, false);

  velocity_controller_parameters_ch1.p = 1.0;
  velocity_controller_parameters_ch1.lpf_time_constant = 0.1;

  holding_position_controller_parameters.p = 2.0;
  holding_position_controller_parameters.i = 0.05;
  holding_position_controller_parameters.limit = 2.0;

  // Configure the position controller
  erasing_position_controller_parameters.p = 3.5;
  erasing_position_controller_parameters.d = 0.5;

  //   Set parameters for holding mode
  position_controller.P = holding_position_controller_parameters.p;
  position_controller.I = holding_position_controller_parameters.i;
  position_controller.D = holding_position_controller_parameters.d;
  position_controller.output_ramp =
      holding_position_controller_parameters.output_ramp;
  position_controller.limit = holding_position_controller_parameters.limit;

  //   We do this ONLY to update the internal low-pass filter for the position
  //   output
  erasing_position_controller_parameters.lpf_time_constant = 0.1;
  tilt_motor.set_position_controller(erasing_position_controller_parameters);
  tilt_motor.set_velocity_controller(velocity_controller_parameters_ch1);
  tilt_motor.set_control_mode(MotorGo::ControlMode::Velocity);
  tilt_motor.set_velocity_limit(FLIPPING_VELOCITY_RAD_PER_SEC);

  enable_motors.set_post_callback(enable_motors_callback);

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

  tilt_motor.enable();
  tilt_motor.loop();

  //   Find the shortest distance to zero position, for holding
  float target_forward =
      get_target(tilt_motor.get_position(), zero_position, 1);
  float target_backward =
      get_target(tilt_motor.get_position(), zero_position, -1);

  if (abs(target_forward) < abs(target_backward))
  {
    target_pos = target_forward;
  }
  else
  {
    target_pos = target_backward;
  }

  Serial.println(target_pos);
}

void loop()
{
  ArduinoOTA.handle();
  float angle = tilt_motor.get_position();

  // Wait until we receive a high on the input pin
  if (digitalRead(input_pin) == HIGH && !erase_begin)
  {
    digitalWrite(output_pin, HIGH);

    // Switch to erasing parameters
    position_controller.P = erasing_position_controller_parameters.p;
    position_controller.I = erasing_position_controller_parameters.i;
    position_controller.D = erasing_position_controller_parameters.d;
    position_controller.output_ramp =
        erasing_position_controller_parameters.output_ramp;
    position_controller.limit = erasing_position_controller_parameters.limit;

    position_controller.reset();

    erase_begin = true;
  }

  if (erase_begin)
  {
    erase_motor_pwm = 255 * (1.5 / config_ch1.power_supply_voltage);

    digitalWrite(CH1_UH, HIGH);
    ledcWrite(7, erase_motor_pwm);

    if (cur_cycle_count >= erase_cycles)
    {
      if (!zeroing)
      {
        zeroing = true;
        zero_start_time = micros();
        target_pos = 0.0;
        position_controller.reset();
      }

      if (micros() - zero_start_time > zero_hold_time)
      {
        zeroing = false;
        erase_begin = false;
        cur_cycle_count = 0;

        Serial.println("Erasing complete");

        // Stop motors
        tilt_motor.disable();

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
        current_pos = false;
      }
      else
      {
        target_pos = get_target(angle, erase_position_2, -1);
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

  float error = target_pos - angle;

  float output = position_controller(error);
  // Print position and error
  tilt_motor.set_target_velocity(output);

  // Update the motors
  tilt_motor.loop();
}
