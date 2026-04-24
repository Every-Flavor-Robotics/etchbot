#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "pid_manager.h"

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

#define FLIPPING_VELOCITY_RAD_PER_SEC 2.0

float zero_position = 0.06;

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& tilt_motor = motorgo_mini.ch0;

MotorGo::MotorConfiguration gm3506_config;
MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

// Position controller parameters
MotorGo::PIDParameters erasing_position_controller_parameters;
MotorGo::PIDParameters holding_position_controller_parameters;
PIDController position_controller(0.0, 0.0, 0.0, 0.0, 0.0);

MotorGo::PIDManager pid_manager;

// configure wifi communications
bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

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

  digitalWrite(42, LOW);

  gm3506_config.pole_pairs = 11;
  gm3506_config.phase_resistance = 5.6;
  gm3506_config.calibration_voltage = 2.0;

  config_ch1.motor_config = gm3506_config;
  config_ch1.power_supply_voltage = 9.0;
  config_ch1.reversed = true;

  tilt_motor.init(config_ch1, false);

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

  tilt_motor.set_control_mode(MotorGo::ControlMode::Voltage);

  enable_motors.set_post_callback(enable_motors_callback);

  pid_manager.add_controller(
      "/holding_controller", holding_position_controller_parameters,
      []()
      {
        position_controller.P = holding_position_controller_parameters.p;
        position_controller.I = holding_position_controller_parameters.i;
        position_controller.D = holding_position_controller_parameters.d;
        position_controller.output_ramp =
            holding_position_controller_parameters.output_ramp;
        position_controller.limit =
            holding_position_controller_parameters.limit;
      });

  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  tilt_motor.enable();
  tilt_motor.loop();

  //   Find the shortest distance to zero position, for holding
  float target_forward =
      get_target(tilt_motor.get_position(), zero_position, 1);
  float target_backward =
      get_target(tilt_motor.get_position(), zero_position, -1);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */
}

void loop_foc(void* pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    float angle = tilt_motor.get_position();

    float error = zero_position - angle;

    float output = position_controller(error);

    // Print position and error
    tilt_motor.set_target_voltage(output);

    tilt_motor.loop();

    esp_task_wdt_reset();
  }
}

void loop() { ArduinoOTA.handle(); }
