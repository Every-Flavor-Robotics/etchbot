#include <Arduino.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include <atomic>
#include <chrono>
#include <vector>

#include "A4988.h"
#include "HTTPClient.h"
#include "MultiDriver.h"
#include "gcode.h"
#include "planner.h"
#include "wifi_gcode_stream.h"

//  Conversion from mm to radians
// #define GEAR_RATIO 30.0 / 66.0  // Motor to knob
#define MICROSTEPS 16
#define GEAR_RATIO (18.0f / 81.0f)  // Motor to knob
#define RAD_PER_MM (0.1858f / GEAR_RATIO)
#define MMPERMIN_TO_RADPERSEC (RAD_PER_MM / 60.0)
#define STEPS_PER_RAD (200.0f / (2.0f * PI)) * MICROSTEPS
#define STEPS_PER_MM (STEPS_PER_RAD * RAD_PER_MM)
#define MMPERMIN_TO_STEPSPERSEC (STEPS_PER_RAD * MMPERMIN_TO_RADPERSEC)

#define ACCELERATION 25000
#define MAX_ACCELERATION 100000
#define UP_DOWN_BACKLASH_RAD (2.0 * STEPS_PER_MM)
#define LEFT_RIGHT_BACKLASH_RAD (1.8 * STEPS_PER_MM)
#define BACKLASH_COMEPSENATION_RADPERSEC 100.0f
// mm * STEPS_PER_MM = rad
#define ERROR_TOLERANCE (0.01 * STEPS_PER_MM)

#define sign(x) ((x) < -0.0001 ? -1 : ((x) > 0.0001 ? 1 : 0))

#define X_LIM 130
#define Y_LIM 89.375

#define LEFT_RIGHT_DIR 35
#define LEFT_RIGHT_STEP 41
#define UP_DOWN_DIR 40
#define UP_DOWN_STEP 39

A4988 left_right(200, LEFT_RIGHT_DIR, LEFT_RIGHT_STEP);
A4988 up_down(200, UP_DOWN_DIR, UP_DOWN_STEP);
MultiDriver controller(left_right, up_down);

TaskHandle_t loop_foc_task;
TickType_t xLastWakeTime;
void loop_foc(void* pvParameters);

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

std::atomic<int> foc_loops(0);

bool enable_flag = false;
bool disable_flag = false;
bool motors_enabled = false;

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
  //   Serial.println("HOMING: WAITING FOR BUTTON PRESS");

  disable_flag = true;
  delay(10);
}

// GCode objects
GCode::GCodeParser* parser;
GCode::WifiGCodeStream* stream;

struct step_command_t
{
  std::atomic<float> left_right_accel;
  std::atomic<float> up_down_accel;
  std::atomic<float> left_right_vel;
  std::atomic<float> up_down_vel;
  std::atomic<long> dx;
  std::atomic<long> dy;
};

struct command_t
{
  std::atomic<bool> ready;
  step_command_t backlash_command;
  step_command_t draw_command;
};

command_t cur_command;

void draw_pre_setup() {}

void draw_setup()
{
  Serial.begin(5000000);
  while (!Serial)
  {
    delay(50);
  }

  stream = new GCode::WifiGCodeStream("192.168.10.15", 50);
  parser = new GCode::GCodeParser(stream, 2000);

  GCode::start_parser(*parser);

  //   xTaskCreatePinnedToCore(
  //       loop_foc,       /* Task function. */
  //       "Loop FOC",     /* name of task. */
  //       10000,          /* Stack size of task */
  //       NULL,           /* parameter of the task */
  //       1,              /* priority of the task */
  //       &loop_foc_task, /* Task handle to keep track of created task */
  //       1);             /* pin task to core 1 */

  left_right.begin(0, MICROSTEPS);
  up_down.begin(0, MICROSTEPS);

  cur_command.ready.store(false);
}

bool backlash_complete = false;
bool command_complete = true;
void loop_foc(void* pvParameters)
{
  //   Serial.print("Loop FOC running on core ");
  //   Serial.println(xPortGetCoreID());

  //   for (;;)
  //   {
  //     // If we're done with the command and a new one is ready
  //     if (command_complete && cur_command.ready.load())
  //     {
  //       step_command_t& command = backlash_complete
  //                                     ? cur_command.draw_command
  //                                     : cur_command.backlash_command;

  //       float left_right_accel = abs(command.left_right_accel.load());
  //       float up_down_accel = abs(command.up_down_accel.load());

  //       // Make sure we don't divide by zero
  //       if (left_right_accel == 0)
  //       {
  //         left_right_accel = 1;
  //       }
  //       if (up_down_accel == 0)
  //       {
  //         up_down_accel = 1;
  //       }

  //       // Load in targets and command them to the motors
  //       left_right.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED,
  //                                  left_right_accel, left_right_accel);

  //       up_down.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED,
  //       up_down_accel,
  //                               up_down_accel);

  //       float left_right_vel = abs(command.left_right_vel.load());
  //       float up_down_vel = abs(command.up_down_vel.load());

  //       if (left_right_vel == 0)
  //       {
  //         left_right_vel = 1;
  //       }

  //       if (up_down_vel == 0)
  //       {
  //         up_down_vel = 1;
  //       }

  //       left_right.setRPM(left_right_vel * 60.0 / (2.0 * PI));
  //       up_down.setRPM(up_down_vel * 60.0 / (2.0 * PI));

  //       long dx = command.dx.load();
  //       long dy = command.dy.load();

  //       //   Parameters
  //       String complete = backlash_complete ? "true" : "false";
  //       Serial.println("---------------");
  //       Serial.println("Backlash complete? " + complete);
  //       Serial.println("backlash dx: " +
  //       String(cur_command.backlash_command.dx)); Serial.println("backlash
  //       dy: " + String(cur_command.backlash_command.dy));
  //       Serial.println("MAIN dx: " + String(cur_command.draw_command.dx));
  //       Serial.println("MAIN dy: " + String(cur_command.draw_command.dy));
  //       Serial.println("dx: " + String(dx)); Serial.println("dy: " +
  //       String(dy)); Serial.println("left_right_accel: " +
  //       String(left_right_accel)); Serial.println("up_down_accel: " +
  //       String(up_down_accel)); Serial.println("left_right_vel: " +
  //       String(left_right_vel)); Serial.println("up_down_vel: " +
  //       String(up_down_vel)); Serial.println("---------------");

  //       //   if dx and dy are both zero, we don't need to move
  //       command_complete = false;

  //       // Only actually command the motion if we have a non-zero dx or dy
  //       //   E
  //       if (dx != 0 || dy != 0)
  //       {
  //         controller.startMove(dx, dy);
  //       }
  //     }

  //     if (!controller.nextAction() && !command_complete)
  //     {
  //       if (!backlash_complete)
  //       {
  //         command_complete = true;
  //         backlash_complete = true;
  //         // Serial.println("Backlash complete");
  //       }
  //       else
  //       {
  //         command_complete = true;
  //         backlash_complete = false;
  //         cur_command.ready.store(false);
  //         // Serial.println("Command complete");
  //       }
  //     }
  //     esp_task_wdt_reset();
  //   }
}

Planner::BacklashCompensatedTrajectoryParameters profile;
Planner::BacklashCompensatedTrajectory cur_trajectory;
float left_right_backlash_offset = 0;
float up_down_backlash_offset = 0;

GCode::MotionCommand command1;
GCode::MotionCommand command2;

// Add pointers to the current and next command
GCode::MotionCommand* current_command = &command1;
bool next_command_ready = false;
GCode::MotionCommand* next_command = &command2;
GCode::MotionCommand* temp_command;

unsigned long previous_loop_time = 0;

float left_right_position = 0;
float up_down_position = 0;

unsigned long last_loop_time = 0;
bool first = true;

float rpm_to_rad_per_sec(float rpm) { return rpm * 2.0 * PI / 60.0; }

bool ready_to_plan = false;
bool draw_loop()
{
  // Safety conditions
  if (left_right_position > (X_LIM + 8) * STEPS_PER_MM ||
      left_right_position < -8 * STEPS_PER_MM ||
      up_down_position > (Y_LIM + 8) * STEPS_PER_MM ||
      up_down_position < -8 * STEPS_PER_MM)
  {
    // Serial.println("Position out of bounds");
    disable_flag = true;
  }

  // If cur_command is not ready, that means we've completed the previous
  // command
  if (!cur_command.ready.load() && next_command_ready)
  {
    Serial.println("Loading next command");
    // Swap the pointers
    temp_command = current_command;
    current_command = next_command;
    next_command = temp_command;
    next_command_ready = false;

    // delay(1000);

    profile.main_profile.x_initial =
        left_right_position - left_right_backlash_offset;
    profile.main_profile.y_initial = up_down_position - up_down_backlash_offset;

    // Print steps and backlash offset
    // Serial.println("X: " + String(left_right_position));
    // Serial.println("Y: " + String(up_down_position));
    // Serial.println("X Offset: " + String(left_right_backlash_offset));
    // Serial.println("Y Offset: " + String(up_down_backlash_offset));

    profile.main_profile.x_final =
        constrain(current_command->x * STEPS_PER_MM, 0, X_LIM * STEPS_PER_MM);
    profile.main_profile.y_final =
        constrain(current_command->y * STEPS_PER_MM, 0, Y_LIM * STEPS_PER_MM);

    profile.main_profile.v_initial = 0;
    profile.main_profile.v_target = 90;
    profile.main_profile.v_final = 0;

    profile.main_profile.a_target = ACCELERATION;

    //   Ignore the rest of the command, run the homing procedure
    if (current_command->home)
    {
      home();

      return true;
    }
    //   Otherwise, generate a trapezoid profile
    else
    {
      //   //   Serial.println("Replan with");
      //   Serial.println("COMMAND X: " + String(current_command->x));
      //   Serial.println("Y: " + String(current_command->y));

      //   delay(3000);
      // Force a replan when we have a new command
      ready_to_plan = true;
    }
  }

  //   Update position based on current velocity
  //   Generate a new profile
  //   Wait for the backlash compensation phase to finish before replanning
  if (ready_to_plan)
  {
    // Serial.println("Updating plan");

    // Assume the motions are complete

    // Update the backlash offsets
    left_right_backlash_offset =
        constrain(left_right_backlash_offset + cur_command.backlash_command.dx,
                  0, LEFT_RIGHT_BACKLASH_RAD);
    up_down_backlash_offset =
        constrain(up_down_backlash_offset + cur_command.backlash_command.dy, 0,
                  UP_DOWN_BACKLASH_RAD);

    left_right_position +=
        cur_command.backlash_command.dx + cur_command.draw_command.dx;
    up_down_position +=
        cur_command.backlash_command.dy + cur_command.draw_command.dy;

    profile.x_current = left_right_position - left_right_backlash_offset;
    profile.y_current = up_down_position - up_down_backlash_offset;

    // Compute current velocity magnitude
    profile.v_current = 0;

    profile.left_right_backlash_offset = left_right_backlash_offset;
    profile.up_down_backlash_offset = up_down_backlash_offset;

    profile.left_right_backlash_distance = LEFT_RIGHT_BACKLASH_RAD;
    profile.up_down_backlash_distance = UP_DOWN_BACKLASH_RAD;

    profile.v_target_backlash = BACKLASH_COMEPSENATION_RADPERSEC;
    // TODO: CHANGE THIS
    profile.a_target_backlash = ACCELERATION;

    profile.backlash_compensation_enabled = true;

    cur_trajectory = Planner::generate_backlash_compensated_profile(
        profile, ERROR_TOLERANCE);

    // We have backlash compensation
    if (cur_trajectory.backlash_compensation_profile.end_delay_delta_us > 0)
    {
      cur_command.backlash_command.left_right_accel.store(
          cur_trajectory.backlash_compensation_profile.a_target *
          cos(cur_trajectory.backlash_compensation_profile.angle));
      cur_command.backlash_command.up_down_accel.store(
          cur_trajectory.backlash_compensation_profile.a_target *
          sin(cur_trajectory.backlash_compensation_profile.angle));

      cur_command.backlash_command.left_right_vel.store(
          cur_trajectory.backlash_compensation_profile.v_target *
          cos(cur_trajectory.backlash_compensation_profile.angle));
      cur_command.backlash_command.up_down_vel.store(
          cur_trajectory.backlash_compensation_profile.v_target *
          sin(cur_trajectory.backlash_compensation_profile.angle));

      cur_command.backlash_command.dx.store(
          cur_trajectory.backlash_compensation_profile.dx);
      cur_command.backlash_command.dy.store(
          cur_trajectory.backlash_compensation_profile.dy);

      Serial.println("---------------");
      Serial.println("Backlash compensation");
      Serial.println("dx: " +
                     String(cur_trajectory.backlash_compensation_profile.dx));

      Serial.println("dy: " +
                     String(cur_trajectory.backlash_compensation_profile.dy));
      Serial.println("---------------");
    }
    else
    {
      // Store zeros in the backlash command
      cur_command.backlash_command.left_right_accel.store(0);
      cur_command.backlash_command.up_down_accel.store(0);
      cur_command.backlash_command.left_right_vel.store(0);
      cur_command.backlash_command.up_down_vel.store(0);
      cur_command.backlash_command.dx.store(0);
      cur_command.backlash_command.dy.store(0);
    }

    cur_command.draw_command.left_right_accel.store(
        cur_trajectory.profile.a_target * cos(cur_trajectory.profile.angle));
    cur_command.draw_command.up_down_accel.store(
        cur_trajectory.profile.a_target * sin(cur_trajectory.profile.angle));

    cur_command.draw_command.left_right_vel.store(
        cur_trajectory.profile.v_target * cos(cur_trajectory.profile.angle));
    cur_command.draw_command.up_down_vel.store(
        cur_trajectory.profile.v_target * sin(cur_trajectory.profile.angle));

    cur_command.draw_command.dx.store(cur_trajectory.profile.dx);
    cur_command.draw_command.dy.store(cur_trajectory.profile.dy);

    cur_command.ready.store(true);

    ready_to_plan = false;
  }

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
      }
    }
  }

  // If we're done with the command and a new one is ready
  if (command_complete && cur_command.ready.load())
  {
    step_command_t& command = backlash_complete ? cur_command.draw_command
                                                : cur_command.backlash_command;

    float left_right_accel = abs(command.left_right_accel.load());
    float up_down_accel = abs(command.up_down_accel.load());

    // Make sure we don't divide by zero
    if (left_right_accel == 0)
    {
      left_right_accel = 1;
    }
    if (up_down_accel == 0)
    {
      up_down_accel = 1;
    }

    // Load in targets and command them to the motors
    left_right.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED,
                               left_right_accel, left_right_accel);

    up_down.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, up_down_accel,
                            up_down_accel);

    float left_right_vel = abs(command.left_right_vel.load());
    float up_down_vel = abs(command.up_down_vel.load());

    if (left_right_vel == 0)
    {
      left_right_vel = 1;
    }

    if (up_down_vel == 0)
    {
      up_down_vel = 1;
    }

    left_right.setRPM(left_right_vel * 60.0 / (2.0 * PI));
    up_down.setRPM(up_down_vel * 60.0 / (2.0 * PI));

    long dx = command.dx.load();
    long dy = command.dy.load();

    //   Parameters
    String complete = backlash_complete ? "true" : "false";
    Serial.println("---------------");
    Serial.println("Backlash complete? " + complete);
    Serial.println("backlash dx: " + String(cur_command.backlash_command.dx));
    Serial.println("backlash dy: " + String(cur_command.backlash_command.dy));
    Serial.println("MAIN dx: " + String(cur_command.draw_command.dx));
    Serial.println("MAIN dy: " + String(cur_command.draw_command.dy));
    Serial.println("dx: " + String(dx));
    Serial.println("dy: " + String(dy));
    Serial.println("left_right_accel: " + String(left_right_accel));
    Serial.println("up_down_accel: " + String(up_down_accel));
    Serial.println("left_right_vel: " + String(left_right_vel));
    Serial.println("up_down_vel: " + String(up_down_vel));
    Serial.println("---------------");

    //   if dx and dy are both zero, we don't need to move
    command_complete = false;

    // Only actually command the motion if we have a non-zero dx or dy
    //   E
    if (dx != 0 || dy != 0)
    {
      controller.startMove(dx, dy);
    }
  }

  if (!controller.nextAction() && !command_complete)
  {
    if (!backlash_complete)
    {
      command_complete = true;
      backlash_complete = true;
      // Serial.println("Backlash complete");
    }
    else
    {
      command_complete = true;
      backlash_complete = false;
      cur_command.ready.store(false);
      // Serial.println("Command complete");
    }
  }

  return false;
}
