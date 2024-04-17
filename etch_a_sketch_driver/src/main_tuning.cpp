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
#define ACCELERATION 10000
#define MAX_ACCELERATION 20000

#define OL_VELOCITY_LIMIT 5000 * MMPERMIN_TO_RADPERSEC

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
// Whether running in OL or CL mode
std::atomic<bool> left_right_ol_mode(false);
std::atomic<bool> up_down_ol_mode(false);

std::atomic<int> foc_loops(0);

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

void home()
{
  Serial.println("HOMING: WAITING FOR BUTTON PRESS");

  disable_flag = true;
  delay(1000);
  while (!motors_enabled)
  {
    delay(100);
  }

  Serial.println("HOMING: BEGINNING");
  //   delay(1000);
  left_right.zero_position();
  up_down.zero_position();
}

unsigned long start_time = 0;

// GCode objects

bool direction = true;
Planner::TrajectoryState state;

float distance = 10;
float command_velocity = 8000;
float voltage_command = 0.2;

void setup()
{
  // Configure onboard button as input
  pinMode(0, INPUT_PULLUP);

  Serial.begin(115200);
  delay(3000);

  MotorGo::MotorConfiguration motor_config;
  motor_config.pole_pairs = 11;
  motor_config.phase_resistance = 0.1;
  motor_config.kv = 500;
  motor_config.current_limit = 10.0;
  motor_config.voltage_limit = 1.6;
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

        distance = planner_lpf_params.p;
        command_velocity = planner_lpf_params.i;
        voltage_command = planner_lpf_params.d;
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
    float local_lr_accel_target = left_right_acceleration_target.load();
    float local_ud_accel_target = up_down_acceleration_target.load();

    // Update LPFs
    float lr_velocity = left_right_velocity_lpf(left_right.get_velocity());
    float ud_velocity = up_down_velocity_lpf(up_down.get_velocity());

    // If in OL mode, simply set the target velocity
    if (left_right_ol_mode.load())
    {
      left_right.set_target_velocity(local_lr_vel_target);
    }
    else
    {
      // Else update controller and set target voltage
      // Update PID controllers
      float lr_command =
          left_right_ff_accel_gain * local_lr_accel_target +
          left_right_ff_velocity_gain * local_lr_vel_target +
          left_right_velocity_pid(local_lr_vel_target - lr_velocity);

      left_right.set_target_voltage(lr_command);
    }

    up_down.set_target_velocity(local_ud_vel_target);

    // if (up_down_ol_mode.load())
    // {
    //   up_down.set_target_velocity(local_ud_vel_target);
    // }
    // else
    // {
    //   float ud_command =
    //       up_down_ff_accel_gain * local_ud_accel_target +
    //       up_down_ff_velocity_gain * local_ud_vel_target +
    //       up_down_velocity_pid(local_ud_vel_target - ud_velocity);

    //   up_down.set_target_voltage(ud_command);
    // }

    left_right.loop();
    up_down.loop();

    foc_loops++;

    esp_task_wdt_reset();
  }
}

Planner::TrapezoidVelocityTrajectory cur_trajectory;
// unsigned long last_print_time = 0;
float last_final_x = 0;
float last_final_y = 0;
bool printed = false;
size_t replan_horizon = 10;
size_t replan_counter = 0;
GCode::MotionCommand current_command;

float previous_velocity_x = 0;
float previous_velocity_y = 0;
unsigned long previous_loop_time = 0;

float average_velocity_error = 0.0;
int velocity_counts = 0;

unsigned long last_loop_time = 0;
float start_position = 0;
void loop()
{
  // We've completed the previous motion AND there are more commands to
  // parse
  if (state.is_complete)
  {
    if (direction)
    {
      current_command.x = distance;
      current_command.y = 0;
      current_command.feedrate = command_velocity;
      current_command.home = false;
    }
    else
    {
      current_command.x = -distance;
      current_command.y = 0;
      current_command.feedrate = command_velocity;
      current_command.home = false;
    }
    direction = !direction;
    // Force a replan when we have a new command
    foc_loops.store(replan_horizon);
    state.is_complete = false;

    // Print command
    Serial.println("Command: " + String(current_command.x) + ", " +
                   String(current_command.y) + ", " +
                   String(current_command.feedrate));

    Serial.println("Average velocity error: " +
                   String(average_velocity_error / velocity_counts));
    average_velocity_error = 0.0;
    velocity_counts = 0;

    home();
    start_position = 0;
  }

  float cur_left_right_velocity =
      planner_left_right_velocity_lpf(left_right.get_velocity());
  float cur_up_down_velocity =
      planner_up_down_velocity_lpf(up_down.get_velocity());

  average_velocity_error += abs(state.v.x - cur_left_right_velocity);
  velocity_counts++;

  if (foc_loops.load() >= replan_horizon)
  {
    // Serial.println("Commands: " + String(current_command.x) + ", " +
    //                String(current_command.y) + ", " +
    //                String(current_command.feedrate));

    unsigned long trajectory_start_time = micros();
    Planner::TrapezoidTrajectoryParameters profile;
    profile.x_initial = left_right.get_position();
    profile.y_initial = up_down.get_position();

    // profile.x_final =
    //     constrain(current_command.x * RAD_PER_MM, 0, X_LIM * RAD_PER_MM);
    profile.y_final = up_down.get_position();

    profile.x_final =
        constrain(current_command.x * RAD_PER_MM, -X_LIM * RAD_PER_MM / 2,
                  X_LIM * RAD_PER_MM / 2);

    // profile.y_final =
    //     constrain(current_command.y * RAD_PER_MM, -Y_LIM * RAD_PER_MM / 2,
    //               Y_LIM * RAD_PER_MM / 2);

    // Compute current velocity magnitude
    float current_velocity =
        sqrt(pow(cur_left_right_velocity, 2) + pow(cur_up_down_velocity, 2));

    // For now, fix initial and final velocities to 0
    profile.v_initial = current_velocity;
    profile.v_final = 0;
    profile.v_target = current_command.feedrate * MMPERMIN_TO_RADPERSEC;
    profile.a_target = ACCELERATION;

    last_final_x = profile.x_final;
    last_final_y = profile.y_final;
    cur_trajectory = Planner::generate_trapezoid_profile(profile);
    cur_trajectory.start_time_us = trajectory_start_time;

    // Switch between control modes based on target velocities in each axis

    float left_right_target_velocity =
        cur_trajectory.v_target * cos(cur_trajectory.angle);
    float up_down_target_velocity =
        cur_trajectory.v_target * sin(cur_trajectory.angle);

    // // If less than velocity limit, but not zero
    // if (abs(left_right_target_velocity) < OL_VELOCITY_LIMIT &&
    //     abs(left_right_target_velocity) > 1)
    // {
    //   //   Serial.println("Left_right: Switching to OL mode");
    //   left_right.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);
    //   left_right_ol_mode.store(true);
    // }
    // else
    // {
    //   Serial.println("Left_right: Switching to CL mode");
    left_right.set_control_mode(MotorGo::ControlMode::Voltage);
    left_right_ol_mode.store(false);
    // }
    // if (abs(up_down_target_velocity) < OL_VELOCITY_LIMIT &&
    //     abs(up_down_target_velocity) > 0.0001)
    // {
    //   //   Serial.println("Up_down: Switching to OL mode");
    //   up_down.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);
    //   up_down_ol_mode.store(true);
    // }
    // else
    // {
    //   //   Serial.println("Up_down: Switching to CL mode");
    //   up_down.set_control_mode(MotorGo::ControlMode::Voltage);
    //   up_down_ol_mode.store(false);
    // }
    replan_counter = 0;
  }

  //   Update velocity commands based on current profile
  unsigned long now = micros();
  state = Planner::compute_trapezoid_velocity_vector(cur_trajectory, now);

  float accel_x =
      (state.v.x - previous_velocity_x) / (now - previous_loop_time) * 1e6;
  float accel_y =
      (state.v.y - previous_velocity_y) / (now - previous_loop_time) * 1e6;

  float accel = sqrt(pow(accel_x, 2) + pow(accel_y, 2));
  if (accel > MAX_ACCELERATION)
  {
    Serial.println("Acceleration too high: " + String(accel));
    // Scale down the velocity
    state.a.x = (MAX_ACCELERATION / accel) * state.a.x;
    state.a.y = (MAX_ACCELERATION / accel) * state.a.y;
    state.v.x = previous_velocity_x +
                (MAX_ACCELERATION / accel) * (state.v.x - previous_velocity_x);

    state.v.y = previous_velocity_y +
                (MAX_ACCELERATION / accel) * (state.v.y - previous_velocity_y);
  }

  left_right_velocity_target.store(state.v.x);
  if (direction)
  {
    up_down_velocity_target.store(voltage_command);
  }
  else
  {
    up_down_velocity_target.store(-voltage_command);
  }

  left_right_acceleration_target.store(state.a.x);
  up_down_acceleration_target.store(state.a.y);

  if (abs(up_down.get_position() - start_position) / RAD_PER_MM > 30)
  {
    Serial.println("Reached this");
    state.is_complete = true;
  }

  previous_velocity_x = state.v.x;
  previous_velocity_y = state.v.y;
}
