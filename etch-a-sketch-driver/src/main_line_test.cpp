#include <Arduino.h>
#include <WebSocketsServer.h>  // Add this for WebSockets
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include <atomic>
#include <chrono>
#include <vector>

#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "gcode.h"
#include "pid_manager.h"
#include "planner.h"
#include "web_server.h"
#include "wifi_gcode_stream.h"

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

//  Conversion from mm to radians
#define GEAR_RATIO 46.0 / 60.0  // Motor to knob
#define RAD_PER_MM 0.1858 / (GEAR_RATIO)
#define MMPERMIN_TO_RADPERSEC RAD_PER_MM / 60.0
#define ACCELERATION 12000
#define MAX_ACCELERATION 15000

#define X_LIM 130
#define Y_LIM 89.375

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
float left_right_ff_accel_gain = 0.0;
float left_right_ff_velocity_gain = 0.0;
float up_down_ff_accel_gain = 0.0;
float up_down_ff_velocity_gain = 0.0;

PIDController left_right_velocity_pid(
    left_right_velocity_pid_params.p, left_right_velocity_pid_params.i,
    left_right_velocity_pid_params.d,
    left_right_velocity_pid_params.output_ramp,
    left_right_velocity_pid_params.limit);

PIDController up_down_velocity_pid(up_down_velocity_pid_params.p,
                                   up_down_velocity_pid_params.i,
                                   up_down_velocity_pid_params.d,
                                   up_down_velocity_pid_params.output_ramp,
                                   up_down_velocity_pid_params.limit);
LowPassFilter left_right_velocity_lpf(0.003);
LowPassFilter up_down_velocity_lpf(0.003);

MotorGo::PIDParameters planner_lpf_params;
LowPassFilter planner_left_right_velocity_lpf(0.0005);
LowPassFilter planner_up_down_velocity_lpf(0.0005);

// declare PID manager object
MotorGo::PIDManager pid_manager;

std::atomic<float> up_down_velocity_target(0.0);
std::atomic<float> left_right_velocity_target(0.0);
std::atomic<float> up_down_acceleration_target(0.0);
std::atomic<float> left_right_acceleration_target(0.0);
std::atomic<float> left_right_gain_schedule(1.0);
std::atomic<float> up_down_gain_schedule(1.0);

bool enable_flag = false;
bool disable_flag = false;
bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

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

float command_angle = 15;
float gain_multiplier = 1.0;
void home()
{
  Serial.println("HOMING: WAITING FOR BUTTON PRESS");

  disable_flag = true;
  delay(1000);
  while (!motors_enabled)
  {
    delay(100);
  }

  // Wait until button is pressed
  //   while (digitalRead(0))
  //   {
  //     delay(100);
  //   }
  Serial.println("HOMING: BEGINNING");
  delay(1000);
  left_right.zero_position();
  up_down.zero_position();
  // Drive negative direction for 20 rads
  // Set target velocity to -5
  //   left_right.zero_position();
  //   while (left_right.get_position() > -30)
  //   {
  //     left_right_velocity_target.store(-5);
  //   }
  //   left_right_velocity_target.store(0);
  //   delay(10);

  //   left_right.zero_position();

  //   // Drive negative direction for 20 rads
  //   // Set target velocity to -5
  //   up_down.zero_position();
  //   while (up_down.get_position() > -20)
  //   {
  //     up_down_velocity_target.store(-5);
  //   }
  //   up_down_velocity_target.store(0);
  //   delay(10);

  //   up_down.zero_position();

  //   // Move 4 mm in the positive x and y direction away from the corner
  //   Planner::TrapezoidTrajectoryParameters zero;
  //   zero.x_initial = 0;
  //   zero.y_initial = 0;
  //   zero.x_final = 4;
  //   zero.y_final = 4;
  //   zero.v_initial = 0;
  //   zero.v_final = 0;
  //   zero.v_target = 50;
  //   zero.a_target = ACCELERATION / 2;

  //   Planner::TrapezoidVelocityTrajectory zero_trajectory =
  //       Planner::generate_trapezoid_profile(zero);

  //   Planner::TrajectoryState zero_state;
  //   zero_state.is_complete = false;
  //   while (!zero_state.is_complete)
  //   {
  //     long now = micros();
  //     zero_state =
  //         Planner::compute_trapezoid_velocity_vector(zero_trajectory, now);

  //     left_right_velocity_target.store(zero_state.v.x);
  //     up_down_velocity_target.store(zero_state.v.y);
  //   }

  //   left_right.zero_position();
  //   up_down.zero_position();
}

unsigned long start_time = 0;

// GCode objects
GCode::GCodeParser* parser;
GCode::WifiGCodeStream* stream;

bool direction = true;
Planner::TrajectoryState state;

Planner::TrapezoidTrajectoryParameters profile_forward;
Planner::TrapezoidTrajectoryParameters profile_backward;

void setup()
{
  // Configure onboard button as input
  pinMode(0, INPUT_PULLUP);

  Serial.begin(115200);
  delay(3000);

  MotorGo::MotorConfiguration motor_config;
  motor_config.pole_pairs = 11;
  motor_config.phase_resistance = 0.1;
  motor_config.kv = 300;
  motor_config.current_limit = 10.0;
  motor_config.voltage_limit = 0.8;
  motor_config.velocity_limit = 300;
  motor_config.calibration_voltage = 0.9;

  // Setup motor parameters
  config_ch0.motor_config = motor_config;
  config_ch0.power_supply_voltage = 17.0;
  config_ch0.reversed = false;

  config_ch1.motor_config = motor_config;
  config_ch1.power_supply_voltage = 17.0;
  config_ch1.reversed = false;

  left_right_velocity_pid_params.p = 0.04;
  left_right_velocity_pid_params.i = 0.001;
  left_right_velocity_pid_params.d = 0.0;
  left_right_velocity_pid_params.lpf_time_constant = 0.007;

  up_down_velocity_pid_params.p = 0.04;
  up_down_velocity_pid_params.i = 0.001;
  up_down_velocity_pid_params.d = 0.0;
  up_down_velocity_pid_params.lpf_time_constant = 0.007;

  planner_lpf_params.lpf_time_constant = 0.0005;

  pid_manager.add_controller(
      "/left_right/velocity", left_right_velocity_pid_params,
      []()
      {
        left_right_velocity_pid.P = left_right_velocity_pid_params.p;
        left_right_velocity_pid.I = 0;
        left_right_velocity_pid.D = 0;

        left_right_ff_velocity_gain = left_right_velocity_pid_params.i;
        left_right_ff_accel_gain = left_right_velocity_pid_params.d / 100.0;

        left_right_velocity_pid.output_ramp =
            left_right_velocity_pid_params.output_ramp;
        left_right_velocity_pid.limit = left_right_velocity_pid_params.limit;

        left_right_velocity_lpf.Tf =
            left_right_velocity_pid_params.lpf_time_constant;
      });

  pid_manager.add_controller(
      "/up_down/velocity", up_down_velocity_pid_params,
      []()
      {
        up_down_velocity_pid.P = up_down_velocity_pid_params.p;
        up_down_velocity_pid.I = 0;
        up_down_velocity_pid.D = 0;

        up_down_ff_velocity_gain = up_down_velocity_pid_params.i;
        up_down_ff_accel_gain = up_down_velocity_pid_params.d / 100.0;

        up_down_velocity_pid.output_ramp =
            up_down_velocity_pid_params.output_ramp;
        up_down_velocity_pid.limit = up_down_velocity_pid_params.limit;

        up_down_velocity_lpf.Tf = up_down_velocity_pid_params.lpf_time_constant;
      });

  pid_manager.add_controller(
      "/planner_lpf", planner_lpf_params,
      []()
      {
        left_right_velocity_lpf.Tf = planner_lpf_params.lpf_time_constant;
        up_down_velocity_lpf.Tf = planner_lpf_params.lpf_time_constant;

        command_angle = planner_lpf_params.p;
        gain_multiplier = planner_lpf_params.i;
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  //   Set closed-loop velocity mode
  left_right.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);
  up_down.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);

  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  //   left_right.zero_position();
  //   up_down.zero_position();

  state.is_complete = false;

  //   Wait until button is pressed
  //   Serial.println("Ready to begin... Press button to start");
  //   while (digitalRead(0))
  //   {
  //     delay(100);
  //   }

  //   delay(1000);

  start_time = micros();

  //   left_right.enable();
  //   up_down.enable();
  home();
}

float target = 0.0;
// Chrono microsChrono(Chrono::MICROS);
unsigned int i = 0;

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
      motors_enabled = true;
    }
    else if (disable_flag)
    {
      Serial.println("Motors are disabled");
      left_right.disable();
      up_down.disable();
      disable_flag = false;
      motors_enabled = false;
    }

    // Get all targets
    float local_lr_vel_target = left_right_velocity_target.load();
    float local_ud_vel_target = up_down_velocity_target.load();
    // float local_lr_accel_target = left_right_acceleration_target.load();
    // float local_ud_accel_target = up_down_acceleration_target.load();

    // // Get gain schedule
    // float local_lr_gain_schedule = left_right_gain_schedule.load();
    // float local_ud_gain_schedule = up_down_gain_schedule.load();

    // // Print targets
    // // Serial.println("Targets: " + String(local_lr_vel_target) + ", " +
    // //                String(local_ud_vel_target) + ", " +
    // //                String(local_lr_accel_target) + ", " +
    // //                String(local_ud_accel_target));

    // // Update PID controllers
    // float lr_velocity = left_right_velocity_lpf(left_right.get_velocity());
    // float ud_velocity = up_down_velocity_lpf(up_down.get_velocity());

    // float lr_command =
    //     left_right_ff_accel_gain * local_lr_accel_target +
    //     left_right_ff_velocity_gain * local_lr_vel_target +
    //     left_right_velocity_pid(local_lr_vel_target - lr_velocity);

    // float ud_command = up_down_ff_accel_gain * local_ud_accel_target +
    //                    up_down_ff_velocity_gain * local_ud_vel_target +
    //                    up_down_velocity_pid(local_ud_vel_target -
    //                    ud_velocity);

    // left_right.set_target_voltage(local_lr_gain_schedule * lr_command);
    // up_down.set_target_voltage(local_ud_gain_schedule * ud_command);

    left_right.set_target_velocity(local_lr_vel_target);
    up_down.set_target_velocity(local_ud_vel_target);

    left_right.loop();
    up_down.loop();

    esp_task_wdt_reset();

    // if (i % 5000 == 1)
    // {
    //   // chrono
    //   Serial.println(microsChrono.elapsed());
    // }
    // i++;
    // microsChrono.restart();
  }
}

Planner::TrapezoidVelocityTrajectory cur_trajectory;
// unsigned long last_print_time = 0;
float last_final_x = 0;
float last_final_y = 0;
bool printed = false;
size_t replan_horizon = 100000;
size_t replan_counter = 0;
GCode::MotionCommand current_command;

// float fake_position_x = 0;
// float fake_position_y = 0;
// float fake_velocity_x = 0;
// float fake_velocity_y = 0;

float previous_velocity_x = 0;
float previous_velocity_y = 0;
unsigned long previous_loop_time = 0;

unsigned long last_loop_time = 0;

float distance_command = 4.85;
float traversed_distance = distance_command;
float velocity_command = 20;
float velocity_deadband = 0.003;

void loop()
{
  if (traversed_distance >= distance_command)
  {
    left_right_velocity_target.store(0);
    up_down_velocity_target.store(0);

    disable_flag = true;
    delay(1000);
    while (!motors_enabled)
    {
      delay(100);
    }

    // Wait until button is pressed
    //   while (digitalRead(0))
    //   {
    //     delay(100);
    //   }
    Serial.println("BEGINNING");
    delay(1000);
    left_right.zero_position();
    up_down.zero_position();
    traversed_distance = 0;

    velocity_command = -velocity_command;
  }
  float left_right_command = velocity_command * cos(command_angle * PI / 180);
  float up_down_command = velocity_command * sin(command_angle * PI / 180);

  //   Print commands
  Serial.println("Commands: " + String(left_right_command) + ", " +
                 String(up_down_command));
  left_right_velocity_target.store(left_right_command);
  up_down_velocity_target.store(up_down_command);

  if (abs(left_right.get_velocity()) < velocity_deadband)
  {
    left_right_gain_schedule.store(gain_multiplier);
  }
  else
  {
    left_right_gain_schedule.store(1.0);
  }
  if (abs(up_down.get_velocity()) < velocity_deadband)
  {
    up_down_gain_schedule.store(gain_multiplier);
  }
  else
  {
    up_down_gain_schedule.store(1.0);
  }
  left_right_acceleration_target.store(0);
  up_down_acceleration_target.store(0);

  traversed_distance =
      sqrt(pow(left_right.get_position(), 2) + pow(up_down.get_position(), 2));

  //   webSocket.loop();  // Handle WebSocket events
}
