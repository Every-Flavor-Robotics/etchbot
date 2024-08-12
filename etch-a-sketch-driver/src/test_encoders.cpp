#include <Arduino.h>
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include <atomic>
#include <chrono>
#include <vector>

#include "configurable.h"

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& left_right = motorgo_mini.ch0;

MotorGo::ChannelConfiguration config_ch0;

std::atomic<float> up_down_velocity_target(0.0);
std::atomic<float> left_right_velocity_target(0.0);
MotorGo::PIDParameters left_right_velocity_pid_params;

bool enable_flag = false;
bool disable_flag = false;
bool motors_enabled = false;

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    enable_flag = true;
    disable_flag = false;
  }
  else
  {
    Serial.println("Disabling motors");
    disable_flag = true;
    enable_flag = false;
  }
}

// Function to print at a maximum frequency
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

unsigned long start_time = 0;

void setup()
{
  // Configure onboard button as input
  pinMode(0, INPUT_PULLUP);

  Serial.begin(115200);
  delay(3000);

  MotorGo::MotorConfiguration motor_config;
  motor_config.pole_pairs = 11;
  //   motor_config.phase_resistance = 0.5;
  motor_config.kv = NOT_SET;
  motor_config.current_limit = 10.0;
  motor_config.voltage_limit = 13.0;
  motor_config.velocity_limit = 300;
  motor_config.calibration_voltage = 0.8;

  // Setup motor parameters
  config_ch0.motor_config = motor_config;
  config_ch0.power_supply_voltage = 12.55;
  config_ch0.reversed = false;

  left_right_velocity_pid_params.p = 0.09;
  left_right_velocity_pid_params.i = 0.0;
  left_right_velocity_pid_params.d = 0.0;
  left_right_velocity_pid_params.lpf_time_constant = 0.007;

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);

  left_right.set_velocity_controller(left_right_velocity_pid_params);

  //   Set closed-loop velocity mode
  left_right.set_control_mode(MotorGo::ControlMode::Velocity);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  left_right.zero_position();

  //   left_right.enable();
  //   up_down.enable();
}

void loop_foc(void* pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    // Service flags
    if (enable_flag)
    {
      Serial.println("Motors are enabled");
      left_right.enable();
      enable_flag = false;
    }
    else if (disable_flag)
    {
      Serial.println("Motors are disabled");
      left_right.disable();
      disable_flag = false;
    }

    left_right.set_target_velocity(left_right_velocity_target.load());

    left_right.loop();

    esp_task_wdt_reset();
  }
}

void loop() {}
