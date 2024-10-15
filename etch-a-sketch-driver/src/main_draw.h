#include <Arduino.h>
#include <WebSocketsServer.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include <atomic>
#include <chrono>
#include <vector>

#include "HTTPClient.h"
#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "gcode.h"
#include "motor_definitions.h"
#include "pid_manager.h"
#include "planner.h"
#include "web_server.h"
#include "wifi_gcode_stream.h"

//  Conversion from mm to radians
#define RAD_PER_MM (MM_TO_KNOB_RAD / GEAR_RATIO)
#define MMPERMIN_TO_RADPERSEC (RAD_PER_MM / 60.0)
#define UP_DOWN_BACKLASH_RAD (UP_DOWN_BACKLASH_MM * RAD_PER_MM)
#define LEFT_RIGHT_BACKLASH_RAD (LEFT_RIGHT_BACKLASH_MM * RAD_PER_MM)

// mm * RAD_PER_MM = rad
#define ERROR_TOLERANCE (ERROR_TOLERANCE_MM * RAD_PER_MM)
#define sign(x) ((x) < -0.0001 ? -1 : ((x) > 0.0001 ? 1 : 0))
#define ENABLED

#define X_LIM (WIDTH + ORIGIN_X)
#define Y_LIM (HEIGHT + ORIGIN_Y)

size_t replan_horizon = 1;

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& left_right = motorgo_mini.ch1;
MotorGo::MotorChannel& up_down = motorgo_mini.ch0;

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
LowPassFilter left_right_velocity_lpf(0.00);
LowPassFilter up_down_velocity_lpf(0.00);

MotorGo::PIDParameters planner_lpf_params;
LowPassFilter planner_left_right_velocity_lpf(0.000);
LowPassFilter planner_up_down_velocity_lpf(0.000);

// declare PID manager object
MotorGo::PIDManager pid_manager;

// Motor definitions
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

String ssid = WIFI_SSID;
String password = WIFI_PASSWORD;

GCode::GCodeParser* parser;
GCode::WifiGCodeStream* stream;

Planner::TrajectoryState state;
float previous_position_x = 0;
float previous_position_y = 0;
float previous_velocity_x = 0;
float previous_velocity_y = 0;
unsigned long previous_loop_time = 0;
bool first = true;
bool complete = false;

Planner::BacklashCompensatedTrajectoryParameters profile;

Planner::BacklashCompensatedTrajectory cur_trajectory;
Planner::Direction left_right_previous_direction = Planner::Direction::BACKWARD;
Planner::Direction up_down_previous_direction = Planner::Direction::BACKWARD;
float left_right_backlash_offset = 0;
float up_down_backlash_offset = 0;

GCode::MotionCommand command1;
GCode::MotionCommand command2;

// Add pointers to the current and next command
GCode::MotionCommand* current_command = &command1;
bool next_command_ready = false;
GCode::MotionCommand* next_command = &command2;
GCode::MotionCommand* temp_command;

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

void enable_motors_callback(bool value)
{
  if (value)
  {
    // Serial.println("Enabling motors");
    enable_flag = true;
    disable_flag = false;
  }
  else
  {
    // Serial.println("Disabling motors");
    disable_flag = true;
    enable_flag = false;
  }
}

void home()
{
  Serial.println("Starting homing procedure");
  left_right.disable();
  up_down.disable();
  left_right.loop();
  up_down.loop();
  delay(1000);
  float homing_speed = 1.0;
  float precision_speed = 0.2;

  left_right.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);
  up_down.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);

  //   Update position first
  for (int i = 0; i < 1000; i++)
  {
    left_right.loop();
    up_down.loop();
    delay(1);
  }

  // How far from the corner to inset the zero
  float inset = 6;

  left_right.enable();
  // Move into the edge of the etch a sketch to zero
  float target = -RAD_PER_MM * (inset + 2);
  //   Move left to the edge
  left_right.set_target_velocity(-homing_speed);
  while (left_right.get_position() > target)
  {
    left_right.loop();
  }
  left_right.set_target_velocity(0.0);
  //   Loop a few times to update encoder readings
  for (int i = 0; i < 1000; i++)
  {
    left_right.loop();
    delayMicroseconds(10);
  }
  //   We've lost the zero since we're slipping at the edge, reset
  left_right.zero_position();
  left_right.loop();

  // Move past the zero point by 1.5 mm
  target = RAD_PER_MM * (inset + LEFT_RIGHT_BACKLASH_MM + 1.);
  // Move right
  left_right.set_target_velocity(1.5 * homing_speed);
  while (left_right.get_position() < target)
  {
    left_right.loop();
  }
  left_right.set_target_velocity(0.0);
  left_right.loop();

  // Move left quickly towards the home
  target = RAD_PER_MM * inset + 0.5;
  left_right.set_target_velocity(-homing_speed);
  while (left_right.get_position() > target)
  {
    left_right.loop();
  }
  left_right.set_target_velocity(0.0);
  left_right.loop();

  // Slow down for high precision for final homing
  target = RAD_PER_MM * inset;
  left_right.set_target_velocity(-precision_speed);
  while (left_right.get_position() > target)
  {
    left_right.loop();
  }
  left_right.disable();
  left_right.loop();

  //   Up down homing

  up_down.enable();
  //   Move into the edge of the etch a sketch to zero
  target = -RAD_PER_MM * (inset + 2);
  //   Move down to the edge
  up_down.set_target_velocity(-homing_speed);
  while (up_down.get_position() > target)
  {
    up_down.loop();
  }
  up_down.set_target_velocity(0.0);
  //   Loop a few times to update encoder readings
  for (int i = 0; i < 1000; i++)
  {
    up_down.loop();
    delayMicroseconds(10);
  }
  //   We've lost the zero since we're slipping at the edge, reset
  up_down.zero_position();
  up_down.loop();

  // Move past the zero point by 1.5 mm
  target = RAD_PER_MM * (inset + UP_DOWN_BACKLASH_MM + 1.);
  // Move up
  up_down.set_target_velocity(1.5 * homing_speed);
  while (up_down.get_position() < target)
  {
    up_down.loop();
  }
  up_down.set_target_velocity(0.0);
  up_down.loop();

  // Move down quickly towards the home
  target = RAD_PER_MM * inset + 0.5;
  up_down.set_target_velocity(-homing_speed);
  while (up_down.get_position() > target)
  {
    up_down.loop();
  }
  up_down.set_target_velocity(0.0);
  up_down.loop();

  // Slow down for high precision for final homing
  target = RAD_PER_MM * inset;
  up_down.set_target_velocity(-precision_speed);
  while (up_down.get_position() > target)
  {
    up_down.loop();
  }
  up_down.disable();
  up_down.loop();

  Serial.println("Homing complete");
}

// void home()
// {
//   Serial.println("Starting homing procedure");
//   left_right.disable();
//   up_down.disable();
//   left_right.loop();
//   up_down.loop();
//   delay(1000);
//   float homing_speed = 1.0;

//   left_right.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);
//   up_down.set_control_mode(MotorGo::ControlMode::VelocityOpenLoop);

//   //   Update position first
//   for (int i = 0; i < 1000; i++)
//   {
//     left_right.loop();
//     up_down.loop();
//     delay(1);
//   }

//   float target = RAD_PER_MM * (ORIGIN_X + 1.0) + LEFT_RIGHT_BACKLASH_RAD;
//   left_right.enable();
//   while (left_right.get_position() < target)
//   {
//     left_right.loop();
//     // Move right
//     left_right.set_target_velocity(homing_speed);
//   }
//   left_right.set_target_velocity(0.0);
//   left_right.loop();

//   target = ORIGIN_X;
//   while (left_right.get_position() > target)
//   {
//     left_right.loop();
//     // Move left
//     left_right.set_target_velocity(-homing_speed);
//   }
//   left_right.set_target_velocity(0.0);
//   left_right.disable();
//   left_right.loop();

//   //   Up down homing
//   target = RAD_PER_MM * (ORIGIN_Y + 1.0) + UP_DOWN_BACKLASH_RAD;
//   up_down.enable();
//   while (up_down.get_position() < target)
//   {
//     up_down.loop();
//     // Move up
//     up_down.set_target_velocity(homing_speed);
//   }
//   up_down.set_target_velocity(0.0);
//   up_down.loop();

//   target = ORIGIN_Y;
//   while (up_down.get_position() > target)
//   {
//     up_down.loop();
//     // Move down
//     up_down.set_target_velocity(-homing_speed);
//   }
//   up_down.set_target_velocity(0.0);
//   up_down.disable();
//   up_down.loop();

//   Serial.println("Homing complete");
// }

void draw_pre_setup()
{
  // Setup motor parameters
  config_ch0.motor_config = GARTTLeftEtch1;
  config_ch0.power_supply_voltage = 15.0;
  config_ch0.reversed = false;

  config_ch1.motor_config = GARTTRightEtch1;
  config_ch1.power_supply_voltage = 15.0;
  config_ch1.reversed = false;

  left_right_velocity_pid_params.p = 0.295;
  left_right_velocity_pid_params.i = 0.0;
  left_right_velocity_pid_params.d = 0.0;
  left_right_velocity_pid_params.lpf_time_constant = 0.00;
  left_right_ff_accel_gain = 0.026 / 100;
  left_right_ff_velocity_gain = 0.011;

  left_right_velocity_pid.P = left_right_velocity_pid_params.p;
  left_right_velocity_pid.I = 0;
  left_right_velocity_pid.D = 0;
  left_right_velocity_lpf.Tf = left_right_velocity_pid_params.lpf_time_constant;

  up_down_velocity_pid_params.p = 0.295;
  up_down_velocity_pid_params.i = 0.0;
  up_down_velocity_pid_params.d = 0.0;
  up_down_velocity_pid_params.lpf_time_constant = 0.00;
  up_down_ff_accel_gain = 0.025 / 100;
  up_down_ff_velocity_gain = 0.011;

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

        // OL_VELOCITY_LIMIT = planner_lpf_params.d;
        //    replan_horizon = planner_lpf_params.i;
        //    FEEDRATE = planner_lpf_params.d;
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  left_right.set_control_mode(MotorGo::ControlMode::Voltage);
  up_down.set_control_mode(MotorGo::ControlMode::Voltage);

  pid_manager.init(ssid, password);
}

void draw_setup()
{
  stream = new GCode::WifiGCodeStream(HOST, 50);
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

  // Let loop start
  delay(1500);

  //   state.is_complete = false;

  left_right.zero_position();
  up_down.zero_position();
  state.is_complete = true;

  previous_position_x = left_right.get_position();
  previous_position_y = up_down.get_position();

  command1.x = previous_position_x;
  command1.y = previous_position_y;

  command2.x = previous_position_x;
  command2.y = previous_position_y;

  profile.main_profile.x_final = previous_position_x;
  profile.main_profile.y_final = previous_position_y;
}

float target = 0.0;
unsigned int i = 0;
MotorGo::ControlMode left_right_cur_control_mode =
    MotorGo::ControlMode::Voltage;
MotorGo::ControlMode up_down_cur_control_mode = MotorGo::ControlMode::Voltage;

void loop_foc(void* pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  //   if (lr_velocity > 150 || ud_velocity > 150)
  //   {
  //     Serial.println("Velocity too high, disabling motors");
  //     disable_flag = true;
  //   }

  while (!complete)
  {
    // Service flags
    if (enable_flag)
    {
      //   Serial.println("Motors are enabled");
      left_right.enable();
      up_down.enable();
      enable_flag = false;
      motors_enabled = true;
    }
    else if (disable_flag)
    {
      //   Serial.println("Motors are disabled");
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

    float lr_velocity = left_right.get_velocity();
    float ud_velocity = up_down.get_velocity();

    // Update LPFs
    // float lr_velocity = left_right_velocity_lpf(left_right.get_velocity());
    // float ud_velocity = up_down_velocity_lpf(up_down.get_velocity());

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

    // Else update controller and set target voltage
    // Update PID controllers
    float lr_command =
        left_right_ff_accel_gain * local_lr_accel_target +
        left_right_ff_velocity_gain * local_lr_vel_target +
        left_right_velocity_pid(local_lr_vel_target - lr_velocity);

    left_right.set_target_voltage(lr_command * left_right_gain);

    float ud_command = up_down_ff_accel_gain * local_ud_accel_target +
                       up_down_ff_velocity_gain * local_ud_vel_target +
                       up_down_velocity_pid(local_ud_vel_target - ud_velocity);

    up_down.set_target_voltage(ud_command * up_down_gain);

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
  //   End task once complete
  vTaskDelete(NULL);
}

bool draw_loop()
{
  // Safety conditions
  if (left_right.get_position() > (X_LIM + 8) * RAD_PER_MM ||
      left_right.get_position() < (ORIGIN_X - 8) * RAD_PER_MM ||
      up_down.get_position() > (Y_LIM + 8) * RAD_PER_MM ||
      up_down.get_position() < (ORIGIN_Y - 8) * RAD_PER_MM)
  {
    Serial.println("Position out of bounds");
    disable_flag = true;
  }

  if (state.is_complete && next_command_ready)
  {
    // Swap the pointers
    temp_command = current_command;
    current_command = next_command;
    next_command = temp_command;
    next_command_ready = false;

    profile.main_profile.x_initial =
        left_right.get_position() - left_right_backlash_offset;
    profile.main_profile.y_initial =
        up_down.get_position() - up_down_backlash_offset;

    profile.main_profile.x_final =
        RAD_PER_MM * constrain(current_command->x, ORIGIN_X, X_LIM);
    profile.main_profile.y_final =
        RAD_PER_MM * constrain(current_command->y, ORIGIN_Y, Y_LIM);

    profile.main_profile.v_initial = 0;
    profile.main_profile.v_target =
        current_command->feedrate * MMPERMIN_TO_RADPERSEC;
    profile.main_profile.v_final = 0;

    profile.main_profile.a_target = ACCELERATION;

    // Print final position in rad and mm
    // Serial.println("Final position: " + String(current_command->x) + ", " +
    //                String(current_command->y) + " mm");
    // delay(2000);

    // Serial.println("Getting next command!");

    //   Ignore the rest of the command, run the homing procedure
    if (current_command->home)
    {
      Serial.println("Homing");
      complete = true;

      home();
      Serial.println(complete);
    }
    //   Otherwise, generate a trapezoid profile
    else
    {
      //   Serial.println("Replan with");
      //   Serial.println("COMMAND X: " + String(current_command->x));
      //   Serial.println("Y: " + String(current_command->y));

      //   delay(3000);
      // Force a replan when we have a new command
      foc_loops.store(replan_horizon);
      state.is_complete = false;
    }
  }

  float cur_left_right_velocity = left_right.get_velocity();
  float cur_up_down_velocity = up_down.get_velocity();

  //   Update position based on current velocity
  //   Generate a new profile
  //   Wait for the backlash compensation phase to finish before replanning
  if (foc_loops.load() >= replan_horizon)
  {
    unsigned long trajectory_start_time = micros();

    // Print positions and backlash offsets
    // Serial.println("Left Right: " + String(left_right.get_position()));
    // Serial.println("Up Down: " + String(up_down.get_position()));
    // Serial.println("Left Right Backlash Offset: " +
    //                String(left_right_backlash_offset));
    // Serial.println("Up Down Backlash Offset: " +
    //                String(up_down_backlash_offset));

    // delay(10000);

    profile.x_current = left_right.get_position() - left_right_backlash_offset;
    profile.y_current = up_down.get_position() - up_down_backlash_offset;

    // Compute current velocity magnitude
    profile.v_current =
        sqrt(pow(cur_left_right_velocity, 2) + pow(cur_up_down_velocity, 2));

    profile.left_right_backlash_offset = left_right_backlash_offset;
    profile.up_down_backlash_offset = up_down_backlash_offset;

    profile.left_right_backlash_distance = LEFT_RIGHT_BACKLASH_RAD;
    profile.up_down_backlash_distance = UP_DOWN_BACKLASH_RAD;

    profile.v_target_backlash = BACKLASH_COMPENSATION_RADPERSEC;
    profile.a_target_backlash = BACKLASH_ACCELERATION;

    profile.left_right_direction_previous = left_right_previous_direction;
    profile.up_down_direction_previous = up_down_previous_direction;
    profile.backlash_compensation_enabled = true;

    cur_trajectory = Planner::generate_backlash_compensated_profile(
        profile, ERROR_TOLERANCE);
    cur_trajectory.backlash_compensation_profile.start_time_us =
        trajectory_start_time;

    // Serial.println(sign(cur_trajectory.v_backlash_left_right));
    // Serial.println(sign(cur_trajectory.v_backlash_up_down));

    // Serial.println("Left Right Backlash Offset: " +
    //                String(left_right_backlash_offset));

    // Serial.println("Up Down Backlash Offset: " +
    //                String(up_down_backlash_offset));

    // Serial.println("Left_right previous " +
    //                String(left_right_previous_direction));
    // Serial.println("Up_down previous " + String(up_down_previous_direction));

    left_right_previous_direction = cur_trajectory.left_right_direction;
    up_down_previous_direction = cur_trajectory.up_down_direction;

    foc_loops.store(0);
  }

  //   Update velocity commands based on current profile
  unsigned long now = micros();
  state = Planner::compute_backlash_compesated_trapezoid_velocity_vector(
      cur_trajectory, now);

  float accel_x =
      (state.v.x - previous_velocity_x) / (now - previous_loop_time) * 1e6;
  float accel_y =
      (state.v.y - previous_velocity_y) / (now - previous_loop_time) * 1e6;

  float accel = sqrt(pow(accel_x, 2) + pow(accel_y, 2));
  if (accel > MAX_ACCELERATION)
  {
    // Serial.println("Acceleration too high: " + String(accel));
    // Scale down the velocity
    state.a.x = (MAX_ACCELERATION / accel) * state.a.x;
    state.a.y = (MAX_ACCELERATION / accel) * state.a.y;
    state.v.x = previous_velocity_x +
                (MAX_ACCELERATION / accel) * (state.v.x - previous_velocity_x);

    state.v.y = previous_velocity_y +
                (MAX_ACCELERATION / accel) * (state.v.y - previous_velocity_y);
  }

  left_right_velocity_target.store(state.v.x);
  left_right_acceleration_target.store(state.a.x);

  up_down_velocity_target.store(state.v.y);
  up_down_acceleration_target.store(state.a.y);

  //   Compute magnitude of speed
  float speed = sqrt(pow(state.v.x, 2) + pow(state.v.y, 2));

  //   Print velocity and acceleration targets
  //   String str =
  //       "Velocity targets: " + String(state.v.x) + ", " + String(state.v.y) +
  //       " Acceleration targets: " + String(state.a.x) + ", " +
  //       String(state.a.y);

  //   freq_println(str, 50);

  previous_velocity_x = state.v.x;
  previous_velocity_y = state.v.y;

  float cur_position_x = left_right.get_position();
  float cur_position_y = up_down.get_position();

  // Update the backlash offsets
  left_right_backlash_offset = constrain(
      left_right_backlash_offset + cur_position_x - previous_position_x, 0,
      LEFT_RIGHT_BACKLASH_RAD);
  up_down_backlash_offset =
      constrain(up_down_backlash_offset + cur_position_y - previous_position_y,
                0, UP_DOWN_BACKLASH_RAD);

  previous_position_x = cur_position_x;
  previous_position_y = cur_position_y;

  //   //   Print position, target position, and target speed
  //   String str = "Position: " + String(cur_position_x) + ", " +
  //                String(cur_position_y) +
  //                " Target: " + String(profile.main_profile.x_final) + ", " +
  //                String(profile.main_profile.y_final) +
  //                " Current Command: " + String(current_command->x) + ", " +
  //                String(current_command->y) +

  //                " Speed: " + String(speed);

  if (!next_command_ready && parser->is_available())
  {
    // Retrieve next command
    GCode::MotionCommandResult result = parser->pop_command_buffer();
    // If the command was successfully retrieved
    if (result.success)
    {
      // Copy data to the next command
      next_command->x = result.command.x;
      next_command->y = result.command.y;
      next_command->z = result.command.z;
      next_command->feedrate = result.command.feedrate;
      next_command->home = result.command.home;
      next_command_ready = true;

      //   Serial.println("Next command ready");
      //   delay(5000);

      if (first)
      {
        first = false;
        Serial.println("Waiting to preprocess all data");
        vTaskDelay(3000 / portTICK_PERIOD_MS);

// if ENABLED is defined, the motors will be enabled
#ifdef ENABLED
        enable_flag = true;
#endif
      }
    }
  }
  return complete;
}
