#include <Arduino.h>
#include <WebSocketsServer.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include <atomic>
#include <chrono>
#include <vector>

#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "gcode.h"
#include "motor_definitions.h"
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
// #define GEAR_RATIO 30.0 / 66.0  // Motor to knob
#define GEAR_RATIO (1 / 2.66666666667)  // Motor to knob
#define RAD_PER_MM (0.1858 / GEAR_RATIO)
#define MMPERMIN_TO_RADPERSEC (RAD_PER_MM / 60.0)
#define ACCELERATION 16000
#define MAX_ACCELERATION 25000
#define UP_DOWN_BACKLASH_RAD (1.35 * RAD_PER_MM)
#define LEFT_RIGHT_BACKLASH_RAD (1.8 * RAD_PER_MM)
#define BACKLASH_COMEPSENATION_RADPERSEC 10.0
// mm * RAD_PER_MM = rad
#define ERROR_TOLERANCE (0.8 * RAD_PER_MM)
#define OL_THRESHOLD (0.2f * RAD_PER_MM)

#define ENABLED

#define sign(x) ((x) < -0.0001 ? -1 : ((x) > 0.0001 ? 1 : 0))

// Rad/s
// #define OL_VELOCITY_LIMIT 15
float OL_VELOCITY_LIMIT = 0;
size_t replan_horizon = 1;

#define X_LIM 130
#define Y_LIM 89.375

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

std::atomic<float> left_right_position_target(0.0);
std::atomic<float> up_down_position_target(0.0);
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
  //   Serial.println("HOMING: WAITING FOR BUTTON PRESS");

  disable_flag = true;
  delay(10);

  esp_restart();

  while (!motors_enabled)
  {
    delay(100);
  }

  //   Serial.println("HOMING: BEGINNING");
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
  while (!Serial)
  {
    delay(50);
  }

  // Setup motor parameters
  config_ch0.motor_config = GARTTLeftTronix;
  config_ch0.power_supply_voltage = 15.0;
  config_ch0.reversed = false;

  config_ch1.motor_config = GARTTRightTronix;
  config_ch1.power_supply_voltage = 15.0;
  config_ch1.reversed = false;

  left_right_velocity_pid_params.p = 0.295;
  left_right_velocity_pid_params.i = 0.0;
  left_right_velocity_pid_params.d = 0.0;
  left_right_velocity_pid_params.lpf_time_constant = 0.00;
  left_right_ff_accel_gain = 0.021 / 100;
  left_right_ff_velocity_gain = 0.011;

  left_right_velocity_pid.P = left_right_velocity_pid_params.p;
  left_right_velocity_pid.I = 0;
  left_right_velocity_pid.D = 0;
  left_right_velocity_lpf.Tf = left_right_velocity_pid_params.lpf_time_constant;

  up_down_velocity_pid_params.p = 0.295;
  up_down_velocity_pid_params.i = 0.0;
  up_down_velocity_pid_params.d = 0.0;
  up_down_velocity_pid_params.lpf_time_constant = 0.00;
  up_down_ff_accel_gain = 0.021 / 100;
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

        OL_VELOCITY_LIMIT = planner_lpf_params.d;
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

  pid_manager.init("NotARobot", "M1crowave!");

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
  //   left_right.enable();
  //   up_down.enable();
  //   home();
  state.is_complete = true;
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

  for (;;)
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

    // If in OL mode, simply set the target velocity
    if (left_right_ol_mode.load())
    {
      if (left_right_cur_control_mode != MotorGo::ControlMode::PositionOpenLoop)
      {
        left_right.set_control_mode(MotorGo::ControlMode::PositionOpenLoop);
        left_right_cur_control_mode = MotorGo::ControlMode::PositionOpenLoop;
      }
      left_right.set_target_position(left_right_position_target.load());

      //   Serial.println("Setting open loop position: " +
      //                  String(left_right_position_target.load()));
    }
    else
    {
      if (left_right_cur_control_mode != MotorGo::ControlMode::Voltage)
      {
        left_right.set_control_mode(MotorGo::ControlMode::Voltage);
        left_right_cur_control_mode = MotorGo::ControlMode::Voltage;
      }

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
      if (up_down_cur_control_mode != MotorGo::ControlMode::PositionOpenLoop)
      {
        up_down.set_control_mode(MotorGo::ControlMode::PositionOpenLoop);
        up_down_cur_control_mode = MotorGo::ControlMode::PositionOpenLoop;
      }

      up_down.set_target_position(up_down_position_target.load());
    }
    else
    {
      if (up_down_cur_control_mode != MotorGo::ControlMode::Voltage)
      {
        up_down.set_control_mode(MotorGo::ControlMode::Voltage);
        up_down_cur_control_mode = MotorGo::ControlMode::Voltage;
      }

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
Planner::Direction left_right_previous_direction = Planner::Direction::BACKWARD;
Planner::Direction up_down_previous_direction = Planner::Direction::BACKWARD;
float left_right_backlash_offset = 0;
float up_down_backlash_offset = 0;

float last_final_x = 0;
float last_final_y = 0;
GCode::MotionCommand command1;
GCode::MotionCommand command2;

// Add pointers to the current and next command
GCode::MotionCommand* current_command = &command1;
bool next_command_ready = false;
GCode::MotionCommand* next_command = &command2;
GCode::MotionCommand* temp_command;

float previous_velocity_x = 0;
float previous_velocity_y = 0;
unsigned long previous_loop_time = 0;

unsigned long last_loop_time = 0;
bool first = true;

int parser_test = 0;

void loop()
{
  // Safety conditions
  if (left_right.get_position() > (X_LIM + 8) * RAD_PER_MM ||
      left_right.get_position() < -8 * RAD_PER_MM ||
      up_down.get_position() > (Y_LIM + 8) * RAD_PER_MM ||
      up_down.get_position() < -8 * RAD_PER_MM)
  {
    // Serial.println("Position out of bounds");
    disable_flag = true;
  }

  if (state.is_complete && next_command_ready)
  {
    // Swap the pointers
    temp_command = current_command;
    current_command = next_command;
    next_command = temp_command;
    next_command_ready = false;

    // delay(1000);

    float rad = left_right.get_position();
    float mm = rad / (RAD_PER_MM);

    // Print final position in rad and mm
    // Serial.println("Final position: " + String(rad, 5) + "rad, " +
    //                String(mm, 5));

    // Serial.println("Getting next command!");

    //   Ignore the rest of the command, run the homing procedure
    if (current_command->home)
    {
      home();
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
  if (foc_loops.load() >= replan_horizon && !state.backlash_compensation_phase)
  {
    unsigned long trajectory_start_time = micros();
    Planner::TrapezoidTrajectoryParameters profile;
    // Serial.println("position: " + String(left_right.get_position()) + "\t" +
    //                left_right_backlash_offset);

    // Serial.println("position y: " + String(up_down.get_position()) + "\t" +
    //                up_down_backlash_offset);
    profile.x_initial = left_right.get_position();
    profile.y_initial = up_down.get_position();
    profile.previous_left_right_direction = left_right_previous_direction;
    profile.previous_up_down_direction = up_down_previous_direction;
    profile.backlash_compensation = true;
    profile.left_right_backlash_compensation_distance = LEFT_RIGHT_BACKLASH_RAD;
    profile.up_down_backlash_compensation_distance = UP_DOWN_BACKLASH_RAD;
    profile.backlash_compensation_velocity = BACKLASH_COMEPSENATION_RADPERSEC;

    profile.x_final =
        constrain(current_command->x * RAD_PER_MM + left_right_backlash_offset,
                  0, X_LIM * RAD_PER_MM);
    profile.y_final =
        constrain(current_command->y * RAD_PER_MM + up_down_backlash_offset, 0,
                  Y_LIM * RAD_PER_MM);

    // Print target in rad and mm

    // String test = "position: " + String(profile.x_final, 5) + "rad, " +
    //               String(profile.x_final / (RAD_PER_MM), 5) + "mm";

    // freq_println(test, 10);

    // Compute current velocity magnitude
    float current_velocity =
        sqrt(pow(cur_left_right_velocity, 2) + pow(cur_up_down_velocity, 2));

    // For now, fix initial and final velocities to 0
    profile.v_initial = current_velocity;
    profile.v_final = 0;
    profile.v_target = current_command->feedrate * MMPERMIN_TO_RADPERSEC;
    profile.a_target = ACCELERATION;

    last_final_x = profile.x_final;
    last_final_y = profile.y_final;
    cur_trajectory =
        Planner::generate_trapezoid_profile(profile, ERROR_TOLERANCE);
    cur_trajectory.start_time_us = trajectory_start_time;

    // delay(10000);

    // trajectory_start_time = micros();

    // Update backlash offset
    left_right_backlash_offset +=
        sign(cur_trajectory.v_backlash_left_right) * LEFT_RIGHT_BACKLASH_RAD;

    up_down_backlash_offset +=
        sign(cur_trajectory.v_backlash_up_down) * UP_DOWN_BACKLASH_RAD;

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
  state = Planner::compute_trapezoid_velocity_vector(cur_trajectory, now,
                                                     OL_THRESHOLD);

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

  if (state.left_right_ol)
  {
    left_right_position_target.store(state.p.x);
  }
  else
  {
    left_right_velocity_target.store(state.v.x);
    left_right_acceleration_target.store(state.a.x);
  }

  if (state.up_down_ol)
  {
    up_down_position_target.store(state.p.y);
  }
  else
  {
    up_down_velocity_target.store(state.v.y);
    up_down_acceleration_target.store(state.a.y);
  }

  left_right_ol_mode.store(state.left_right_ol);
  up_down_ol_mode.store(state.up_down_ol);

  previous_velocity_x = state.v.x;
  previous_velocity_y = state.v.y;

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
}
