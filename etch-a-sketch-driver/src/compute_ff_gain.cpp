#include <Arduino.h>
#include <WebSocketsServer.h>  // Add this for WebSockets
#include <esp_task_wdt.h>

#include <atomic>
#include <vector>

#include "pid_manager.h"
#include "planner.h"

//  Conversion from mm to radians
#define LEFT_RIGHT_RAD_PER_MM 0.2364
#define UP_DOWN_RAD_PER_MM 0.1858
#define ACCELERATION 50

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& left_right = motorgo_mini.ch0;
MotorGo::MotorChannel& up_down = motorgo_mini.ch1;

MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

// Assume that both channels have the same PID parameters
MotorGo::PIDParameters left_right_velocity_pid_params;
MotorGo::PIDParameters up_down_velocity_pid_params;

// declare PID manager object
MotorGo::PIDManager pid_manager;

std::atomic<float> up_down_voltage_target(0.0);
std::atomic<float> left_right_voltage_target(0.0);
std::atomic<int> up_down_sign(1);
std::atomic<int> left_right_sign(1);

bool enable_flag = false;
bool disable_flag = false;
bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

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
  motor_config.voltage_limit = 12.0;
  motor_config.velocity_limit = 300;
  motor_config.calibration_voltage = 0.8;

  // Setup motor parameters
  config_ch0.motor_config = motor_config;
  config_ch0.power_supply_voltage = 12.55;
  config_ch0.reversed = false;

  config_ch1.motor_config = motor_config;
  config_ch1.power_supply_voltage = 12.55;
  config_ch1.reversed = false;

  pid_manager.add_controller(
      "/left_right/velocity", left_right_velocity_pid_params,
      []()
      { left_right_voltage_target.store(left_right_velocity_pid_params.p); });

  pid_manager.add_controller(
      "/up_down/velocity", up_down_velocity_pid_params,
      []()
      {
        up_down_voltage_target.store(up_down_velocity_pid_params.p);
        up_down.set_velocity_controller(up_down_velocity_pid_params);
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  left_right.set_velocity_controller(left_right_velocity_pid_params);
  up_down.set_velocity_controller(up_down_velocity_pid_params);

  //   Set closed-loop velocity mode
  left_right.set_control_mode(MotorGo::ControlMode::Voltage);
  up_down.set_control_mode(MotorGo::ControlMode::Voltage);

  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  left_right.zero_position();
  up_down.zero_position();

  //   Wait until button is pressed
  Serial.println("Ready to begin... Press button to start");
  //   while (digitalRead(0))
  //   {
  //     delay(100);
  //   }

  //   left_right.enable();
  //   up_down.enable();
}

float target = 0.0;

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
      up_down.enable();
      enable_flag = false;
    }
    else if (disable_flag)
    {
      Serial.println("Motors are disabled");
      left_right.disable();
      up_down.disable();
      disable_flag = false;
    }

    left_right.set_target_voltage(left_right_sign.load() *
                                  left_right_voltage_target.load());
    up_down.set_target_voltage(up_down_sign.load() *
                               up_down_voltage_target.load());

    left_right.loop();
    up_down.loop();

    esp_task_wdt_reset();
  }
}

float max_distance = 18;
bool direction = true;

float left_right_avg_velocity = 0.0;
float up_down_avg_velocity = 0.0;

int count = 0.0;
unsigned long start_time = 0;
void loop()
{
  if (direction)
  {
    if (left_right.get_position() >= max_distance)
    {
      direction = false;
      left_right_sign.store(-1);
      up_down_sign.store(-1);

      //   Print average velocity
      Serial.print(String(left_right_avg_velocity / count, 5));
      Serial.print(",");
      //   Serial.println("Up-Down average velocity: " +
      //                  String(up_down_avg_velocity / count));

      count = 0.0;
      start_time = micros();
      left_right_avg_velocity = 0.0;
      up_down_avg_velocity = 0.0;
    }
  }
  else
  {
    if (left_right.get_position() <= 0)
    {
      direction = true;
      left_right_sign.store(1);
      up_down_sign.store(1);

      //   Print average velocity
      Serial.print(String(-1 * left_right_avg_velocity / count, 5));
      Serial.print(",");
      //   Serial.println("Up-Down average velocity: " +
      //                  String(up_down_avg_velocity / count));

      count = 0.0;
      start_time = micros();

      left_right_avg_velocity = 0.0;
      up_down_avg_velocity = 0.0;
    }
  }
  unsigned long now = micros();

  //   Compute average if 0.1 seconds have passed
  if (now - start_time > 5e4)
  {
    left_right_avg_velocity += left_right.get_velocity();
    up_down_avg_velocity += up_down.get_velocity();

    count += 1;
  }
}
