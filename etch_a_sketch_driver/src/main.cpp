#include <Arduino.h>
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include "configurable.h"
#include "pid_manager.h"
#include "web_server.h"

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
MotorGo::PIDParameters left_right_position_pid_params;
MotorGo::PIDParameters up_down_position_pid_params;

// declare PID manager object
MotorGo::PIDManager pid_manager;

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

struct VelocityVector
{
  float x;
  float y;
};

struct TrajectoryState
{
  VelocityVector v;
  bool is_complete = false;
};

// Define a struct to hold the parameters of a trapezoidal profile
struct TrapezoidalProfile
{
  // Start time of the profile
  long start_time_us = 0;
  //   Time from start for constant acceleration
  long acceleration_time_delta_us;
  //   Time from start for constant velocity
  long coast_end_time_delta_us;
  //   Time from start for constant deceleration
  long end_time_delta_us;

  float angle;
  float v_initial;
  float v_target;
  float v_final;

  float a_target;
};

struct TrapezoidArgs
{
  float x_initial;
  float y_initial;
  float v_initial;
  float x_final;
  float y_final;
  float v_final;
  float v_target;
  float a_target;
};

TrapezoidalProfile generate_trapezoidal_profile(TrapezoidArgs args)
{
  // Calculate the distance between the initial and final points
  float dx = args.x_final - args.x_initial;
  float dy = args.y_final - args.y_initial;
  float d_total = sqrt(dx * dx + dy * dy);

  // Compute distance over constant acceleration/deceleration region
  // x_f = (v_f^2 - v_i^2) / (2 * a)
  float d_accel =
      0.5 * (args.v_target * args.v_target - args.v_initial * args.v_initial) /
      args.a_target;

  float d_decel =
      0.5 * (args.v_final * args.v_final - args.v_target * args.v_target) /
      -args.a_target;

  // Confirm that the distance is long enough to reach the target velocity
  if (d_total < d_accel + d_decel)
  {
    Serial.println("Distance is too short to reach target velocity");
    // Compute a new target velocity that can be reached
    // v_f = sqrt(2 * a * x_f + v_i^2)
    args.v_target =
        sqrt(2 * args.a_target * d_total + args.v_initial * args.v_initial +
             args.v_final * args.v_final) /
        2;

    // Recompute the distance over the constant acceleration/deceleration region
    d_accel =
        0.5 *
        (args.v_target * args.v_target - args.v_initial * args.v_initial) /
        args.a_target;

    d_decel = 0.5 *
              (args.v_final * args.v_final - args.v_target * args.v_target) /
              -args.a_target;
  }

  TrapezoidalProfile profile;

  //   Compute the time to reach the target velocity
  // v_f = v_i + a * t
  float t_accel = (args.v_target - args.v_initial) / args.a_target;
  float t_decel = (args.v_final - args.v_target) / -args.a_target;

  float t_coast = (d_total - d_accel - d_decel) / args.v_target;

  profile.acceleration_time_delta_us = (long)t_accel * 1e6;
  profile.coast_end_time_delta_us =
      profile.acceleration_time_delta_us + (long)t_coast * 1e6;
  profile.end_time_delta_us =
      profile.coast_end_time_delta_us + (long)t_decel * 1e6;

  profile.v_initial = args.v_initial;
  profile.v_target = args.v_target;
  profile.v_final = args.v_final;

  profile.a_target = args.a_target;

  profile.angle = atan2(dy, dx);

  //   Print all the parameters
  Serial.println("Acceleration time: " + String(t_accel));
  Serial.println("Coast time: " + String(t_coast));
  Serial.println("Deceleration time: " + String(t_decel));
  Serial.println("Acceleration distance: " + String(d_accel));
  Serial.println("Coast distance: " + String(d_total - d_accel - d_decel));
  Serial.println("Deceleration distance: " + String(d_decel));
  Serial.println("Total distance: " + String(d_total));
  Serial.println("Initial velocity: " + String(args.v_initial));
  Serial.println("Target velocity: " + String(args.v_target));
  Serial.println("Final velocity: " + String(args.v_final));
  Serial.println("Target angle: " + String(profile.angle));
  Serial.println("Acceleration time delta: " +
                 String(profile.acceleration_time_delta_us));
  Serial.println("Coast end time delta: " +
                 String(profile.coast_end_time_delta_us));
  Serial.println("End time delta: " + String(profile.end_time_delta_us));

  return profile;
}

// Compute the velocity vector at a given time
TrajectoryState compute_velocity_vector(TrapezoidalProfile& profile,
                                        long time_us)
{
  TrajectoryState state;
  float v_target;
  //   Compute t relative to the start time
  float t_us = time_us - profile.start_time_us;
  float t = t_us / 1e6;

  // Trajectory has not started, set the start time and initial velocity
  if (profile.start_time_us == 0)
  {
    Serial.println("Starting profile");
    profile.start_time_us = time_us;
    v_target = profile.v_initial;
  }

  // Constant acceleration phase
  else if (t_us < profile.acceleration_time_delta_us)
  {
    //   Acceleration phase
    v_target = profile.v_initial + profile.a_target * t;
  }
  //   Constant velocity phase
  else if (t_us < profile.coast_end_time_delta_us)
  {
    //   Constant velocity phase
    v_target = profile.v_target;
  }
  //   Deceleration phase
  else if (t_us < profile.end_time_delta_us)
  {
    //   Deceleration phase
    v_target = profile.v_target -
               profile.a_target * (t - profile.coast_end_time_delta_us / 1e6);
  }
  else
  {
    //   Profile is complete
    v_target = profile.v_final;
    state.is_complete = true;
  }

  state.v.x = v_target * cos(profile.angle);
  state.v.y = v_target * sin(profile.angle);
  return state;
}

TrapezoidArgs forward;
TrapezoidArgs backward;

bool direction = true;
TrajectoryState state;

void setup()
{
  Serial.begin(115200);
  delay(3000);

  MotorGo::MotorConfiguration motor_config;
  motor_config.pole_pairs = 11;
  motor_config.phase_resistance = 0.5;
  //   Underestimating, I think
  motor_config.kv = NOT_SET;
  motor_config.current_limit = 10.0;
  motor_config.voltage_limit = 12.0;
  motor_config.velocity_limit = 300;
  motor_config.calibration_voltage = 0.8;

  // Setup motor parameters
  config_ch0.motor_config = motor_config;
  config_ch0.power_supply_voltage = 7.5;
  config_ch0.reversed = false;

  config_ch1.motor_config = motor_config;
  config_ch1.power_supply_voltage = 7.5;
  config_ch1.reversed = false;

  pid_manager.add_controller(
      "/left_right/velocity", left_right_velocity_pid_params,
      []()
      {
        left_right_velocity_pid_params.output_ramp =
            left_right_velocity_pid_params.i;
        left_right_velocity_pid_params.i = 0.0;

        left_right_velocity_pid_params.limit = left_right_velocity_pid_params.d;
        left_right_velocity_pid_params.d = 0.0;

        left_right.set_velocity_controller(left_right_velocity_pid_params);
      });

  pid_manager.add_controller(
      "/up_down/velocity", up_down_velocity_pid_params,
      []()
      {
        up_down_velocity_pid_params.output_ramp = up_down_velocity_pid_params.i;
        up_down_velocity_pid_params.i = 0.0;

        up_down_velocity_pid_params.limit = up_down_velocity_pid_params.d;
        up_down_velocity_pid_params.d = 0.0;

        up_down.set_velocity_controller(up_down_velocity_pid_params);
      });

  pid_manager.add_controller(
      "/left_right/position", left_right_position_pid_params,
      []()
      {
        left_right_position_pid_params.output_ramp =
            left_right_position_pid_params.i;
        left_right_position_pid_params.i = 0.0;

        left_right_position_pid_params.limit = left_right_position_pid_params.d;
        left_right_position_pid_params.d = 0.0;

        left_right.set_position_controller(left_right_position_pid_params);
      });

  pid_manager.add_controller(
      "/up_down/position", up_down_position_pid_params,
      []()
      {
        up_down_position_pid_params.output_ramp = up_down_position_pid_params.i;
        up_down_position_pid_params.i = 0.0;

        up_down_position_pid_params.limit = up_down_position_pid_params.d;
        up_down_position_pid_params.d = 0.0;

        up_down.set_position_controller(up_down_position_pid_params);
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  left_right.set_velocity_controller(left_right_velocity_pid_params);
  up_down.set_velocity_controller(up_down_velocity_pid_params);

  left_right.set_position_controller(left_right_position_pid_params);
  up_down.set_position_controller(up_down_position_pid_params);

  //   Set closed-loop velocity mode
  left_right.set_control_mode(MotorGo::ControlMode::Velocity);
  up_down.set_control_mode(MotorGo::ControlMode::Velocity);

  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  //   xTaskCreatePinnedToCore(
  //       loop_foc,       /* Task function. */
  //       "Loop FOC",     /* name of task. */
  //       10000,          /* Stack size of task */
  //       NULL,           /* parameter of the task */
  //       1,              /* priority of the task */
  //       &loop_foc_task, /* Task handle to keep track of created task */
  //       1);             /* pin task to core 1 */

  left_right.zero_position();
  up_down.zero_position();

  //   Define profiles
  forward.x_initial = 0.0;
  forward.y_initial = 0.0;
  forward.v_initial = 0.0;
  forward.x_final = 5.0;
  forward.y_final = 5.0;
  forward.v_final = 0.0;
  forward.v_target = 4.0;
  forward.a_target = 0.5;

  backward.x_initial = 5.0;
  backward.y_initial = 5.0;
  backward.v_initial = 0.0;
  backward.x_final = 0.0;
  backward.y_final = 0.0;
  backward.v_final = 0.0;
  backward.v_target = 4.0;
  backward.a_target = 0.5;

  state.is_complete = true;

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

    left_right.loop();
    up_down.loop();

    esp_task_wdt_reset();
  }
}

long last_time = 0;
TrapezoidalProfile profile;
void loop()
{
  if (state.is_complete)
  {
    if (direction)
    {
      forward.x_initial = left_right.get_position();
      forward.y_initial = up_down.get_position();
      profile = generate_trapezoidal_profile(forward);
    }
    else
    {
      backward.x_initial = left_right.get_position();
      backward.y_initial = up_down.get_position();
      profile = generate_trapezoidal_profile(backward);
    }
    state.is_complete = false;
    direction = !direction;
  }

  long now = micros();
  state = compute_velocity_vector(profile, now);

  left_right.set_target_velocity(state.v.x);
  up_down.set_target_velocity(state.v.y);

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

  left_right.loop();
  up_down.loop();

  //   Print target velocity
  freq_println(
      "Target velocity: " + String(state.v.x - left_right.get_velocity()) +
          ", " + String(state.v.y - up_down.get_velocity()) + ", " +
          String(left_right.get_position()) + ", " +
          String(up_down.get_position()),
      20);
}