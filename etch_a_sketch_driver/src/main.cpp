#include <Arduino.h>
#include <WebSocketsServer.h>
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
#define GEAR_RATIO 30.0 / 66.0  // Motor to knob
#define RAD_PER_MM 0.1858 / (GEAR_RATIO)
#define MMPERMIN_TO_RADPERSEC RAD_PER_MM / 60.0
#define ACCELERATION 10000
#define MAX_ACCELERATION 20000
// mm * RAD_PER_MM = rad
#define ERROR_TOLERANCE 1.0 * RAD_PER_MM

// Rad/s
// #define OL_VELOCITY_LIMIT 15
float OL_VELOCITY_LIMIT = 0;
size_t replan_horizon = 1;

// float FEEDRATE = 8000;

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
LowPassFilter planner_left_right_velocity_lpf(0.000);
LowPassFilter planner_up_down_velocity_lpf(0.000);

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
float left_right_gain_multiplier = 1.0;
float up_down_gain_multiplier = 1.0;

std::atomic<int> foc_loops(0);

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
      // streaming)/
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
  delay(1000);
  left_right.zero_position();
  up_down.zero_position();
}

unsigned long start_time = 0;

// GCode objects
GCode::GCodeParser* parser;
GCode::WifiGCodeStream* stream;

bool direction = true;
Planner::TrajectoryState state;

void setup()
{
  // Configure onboard button as input
  pinMode(0, INPUT_PULLUP);

  Serial.begin(5000000);
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

  left_right_velocity_pid_params.p = 0.26;
  left_right_velocity_pid_params.i = 0.0;
  left_right_velocity_pid_params.d = 0.0;
  left_right_velocity_pid_params.lpf_time_constant = 0.00;
  left_right_ff_accel_gain = 0.04 / 100;
  left_right_ff_velocity_gain = 0.025;

  left_right_velocity_pid.P = left_right_velocity_pid_params.p;
  left_right_velocity_pid.I = 0;
  left_right_velocity_pid.D = 0;
  left_right_velocity_lpf.Tf = left_right_velocity_pid_params.lpf_time_constant;

  up_down_velocity_pid_params.p = 0.26;
  up_down_velocity_pid_params.i = 0.0;
  up_down_velocity_pid_params.d = 0.0;
  up_down_velocity_pid_params.lpf_time_constant = 0.00;
  up_down_ff_accel_gain = 0.04 / 100;
  up_down_ff_velocity_gain = 0.025;

  up_down_velocity_pid.P = up_down_velocity_pid_params.p;
  up_down_velocity_pid.I = 0;
  up_down_velocity_pid.D = 0;
  up_down_velocity_lpf.Tf = up_down_velocity_pid_params.lpf_time_constant;

  planner_lpf_params.lpf_time_constant = 0.000;

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
        planner_left_right_velocity_lpf.Tf =
            planner_lpf_params.lpf_time_constant;
        planner_up_down_velocity_lpf.Tf = planner_lpf_params.lpf_time_constant;

        up_down_gain_multiplier = planner_lpf_params.p;
        left_right_gain_multiplier = planner_lpf_params.i;

        OL_VELOCITY_LIMIT = planner_lpf_params.d;
        //    replan_horizon = planner_lpf_params.i;
        //    FEEDRATE = planner_lpf_params.d;
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  //   Set closed-loop velocity mode
  left_right.set_control_mode(MotorGo::ControlMode::Voltage);
  up_down.set_control_mode(MotorGo::ControlMode::Voltage);

  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  // Start the WebSocket server
  //   webSocket.begin();
  //   webSocket.onEvent(webSocketEvent);

  stream = new GCode::WifiGCodeStream("192.168.10.15", 50);
  parser = new GCode::GCodeParser(stream, 2000);

  GCode::start_parser(*parser);

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  state.is_complete = false;

  start_time = micros();

  left_right.zero_position();
  up_down.zero_position();
  left_right.enable();
  up_down.enable();
  //   home();
  state.is_complete = true;
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

    float left_right_gain = 1.0;
    if (lr_velocity < 0.01)
    {
      left_right_gain = left_right_gain_multiplier;
    }
    float up_down_gain = 1.0;
    if (ud_velocity < 0.01)
    {
      up_down_gain = up_down_gain_multiplier;
    }

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

      left_right.set_target_voltage(lr_command * left_right_gain);
    }

    if (up_down_ol_mode.load())
    {
      up_down.set_target_velocity(local_ud_vel_target);
    }
    else
    {
      float ud_command =
          up_down_ff_accel_gain * local_ud_accel_target +
          up_down_ff_velocity_gain * local_ud_vel_target +
          up_down_velocity_pid(local_ud_vel_target - ud_velocity);

      up_down.set_target_voltage(ud_command * up_down_gain);
    }

    // // Print targets
    // // Serial.println("Targets: " + String(local_lr_vel_target) + ", " +
    // //                String(local_ud_vel_target) + ", " +
    // //                String(local_lr_accel_target) + ", " +
    // //                String(local_ud_accel_target));

    left_right.loop();
    up_down.loop();

    foc_loops++;

    esp_task_wdt_reset();
  }
}

Planner::TrapezoidVelocityTrajectory cur_trajectory;
float last_final_x = 0;
float last_final_y = 0;
GCode::MotionCommand current_command;

float previous_velocity_x = 0;
float previous_velocity_y = 0;
unsigned long previous_loop_time = 0;

unsigned long last_loop_time = 0;
bool first = true;

void loop()
{
  // Safety conditions
  if (left_right.get_position() > (X_LIM + 8) * RAD_PER_MM ||
      left_right.get_position() < -8 * RAD_PER_MM ||
      up_down.get_position() > (Y_LIM + 8) * RAD_PER_MM ||
      up_down.get_position() < -8 * RAD_PER_MM)
  {
    Serial.println("Position out of bounds");
    disable_flag = true;
  }

  if (state.is_complete && parser->is_available())
  {
    // Retrieve next command
    GCode::MotionCommandResult result = parser->pop_command_buffer();
    // If the command was successfully retrieved
    if (result.success)
    {
      current_command = result.command;
      //   Ignore the rest of the command, run the homing procedure
      if (current_command.home)
      {
        home();
      }
      //   Otherwise, generate a trapezoid profile
      else
      {
        // Force a replan when we have a new command
        foc_loops.store(replan_horizon);
        state.is_complete = false;
        if (first)
        {
          first = false;
          vTaskDelay(3000 / portTICK_PERIOD_MS);
        }

        // Print command
        // Serial.println("Command: " + String(current_command.x) + ", " +
        //                String(current_command.y) + ", " +
        //                String(current_command.feedrate));

        // Serial.println("Next command ready");
        // while (digitalRead(0))
        // {
        //   delay(100);
        // }
        // delay(1000);
        // Serial.println("Starting next");
      }
    }
  }

  float cur_left_right_velocity = left_right.get_velocity();
  float cur_up_down_velocity = up_down.get_velocity();

  //   Update position based on current velocity
  //   Generate a new profile
  if (foc_loops.load() >= replan_horizon)
  {
    unsigned long trajectory_start_time = micros();
    Planner::TrapezoidTrajectoryParameters profile;
    profile.x_initial = left_right.get_position();
    profile.y_initial = up_down.get_position();

    profile.x_final =
        constrain(current_command.x * RAD_PER_MM, 0, X_LIM * RAD_PER_MM);
    profile.y_final =
        constrain(current_command.y * RAD_PER_MM, 0, Y_LIM * RAD_PER_MM);

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
    cur_trajectory =
        Planner::generate_trapezoid_profile(profile, ERROR_TOLERANCE);
    cur_trajectory.start_time_us = trajectory_start_time;

    // Switch between control modes based on target velocities in each axis
    float left_right_target_velocity =
        cur_trajectory.v_target * cos(cur_trajectory.angle);
    float up_down_target_velocity =
        cur_trajectory.v_target * sin(cur_trajectory.angle);

    // If less than velocity limit, but not zero
    if (abs(left_right_target_velocity) < OL_VELOCITY_LIMIT &&
        abs(left_right_target_velocity) > 1)
    {
      //   Serial.println("Left_right: Switching to OL mode");
      left_right.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);
      left_right_ol_mode.store(true);
    }
    else
    {
      //   Serial.println("Left_right: Switching to CL mode");
      left_right.set_control_mode(MotorGo::ControlMode::Voltage);
      left_right_ol_mode.store(false);
    }
    if (abs(up_down_target_velocity) < OL_VELOCITY_LIMIT &&
        abs(up_down_target_velocity) > 0.0001)
    {
      //   Serial.println("Up_down: Switching to OL mode");
      up_down.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);
      up_down_ol_mode.store(true);
    }
    else
    {
      //   Serial.println("Up_down: Switching to CL mode");
      up_down.set_control_mode(MotorGo::ControlMode::Voltage);
      up_down_ol_mode.store(false);
    }
    // Serial.println("Command: " + String(command.x) + ", " +
    //                String(command.y) + ", " + String(command.feedrate));
    // // Print all profile parameters
    // Serial.println("Profile parameters:");
    // Serial.println("Initial position: " + String(profile.x_initial) + ","
    // +
    //                String(profile.y_initial));
    // Serial.println("Final position: " + String(profile.x_final) + "," +
    //                String(profile.y_final));

    // Serial.println("Initial velocity: " + String(profile.v_initial));
    // Serial.println("Target velocity:" + String(profile.v_target));
    // Serial.println("Final velocity: " + String(profile.v_final));
    // Serial.println("Acceleration: " + String(profile.a_target));

    // Serial.println("Accel time: " +
    //                String(cur_trajectory.acceleration_time_delta_us));
    // Serial.println("Coast time: " +
    //                String(cur_trajectory.coast_end_time_delta_us));
    // Serial.println("Decel time: " +
    // String(cur_trajectory.end_time_delta_us));

    // Serial.println("End time: " + String(cur_trajectory.end_time_delta_us));

    foc_loops.store(0);
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
  up_down_velocity_target.store(state.v.y);

  left_right_acceleration_target.store(state.a.x);
  up_down_acceleration_target.store(state.a.y);

  previous_velocity_x = state.v.x;
  previous_velocity_y = state.v.y;

}
