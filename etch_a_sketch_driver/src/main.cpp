#include <Arduino.h>
#include <WebSocketsServer.h>  // Add this for WebSockets
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include <atomic>
#include <vector>

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
#define RAD_PER_MM 0.1858
#define RAD_PER_MM 0.1858
#define MMPERMIN_TO_RADPERSEC RAD_PER_MM / 60.0
#define ACCELERATION 5000

#define X_LIM 15 * RAD_PER_MM
#define Y_LIM 10.3125 * RAD_PER_MM

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

std::atomic<float> up_down_velocity_target(0.0);
std::atomic<float> left_right_velocity_target(0.0);

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

  config_ch1.motor_config = motor_config;
  config_ch1.power_supply_voltage = 12.55;
  config_ch1.reversed = false;

  left_right_velocity_pid_params.p = 0.09;
  left_right_velocity_pid_params.i = 0.0;
  left_right_velocity_pid_params.d = 0.0;
  left_right_velocity_pid_params.lpf_time_constant = 0.007;

  up_down_velocity_pid_params.p = 0.09;
  up_down_velocity_pid_params.i = 0.0;
  up_down_velocity_pid_params.d = 0.0;
  up_down_velocity_pid_params.lpf_time_constant = 0.007;

  pid_manager.add_controller(
      "/left_right/velocity", left_right_velocity_pid_params,
      []()
      { left_right.set_velocity_controller(left_right_velocity_pid_params); });

  pid_manager.add_controller(
      "/up_down/velocity", up_down_velocity_pid_params,
      []() { up_down.set_velocity_controller(up_down_velocity_pid_params); });

  enable_motors.set_post_callback(enable_motors_callback);

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  left_right.set_velocity_controller(left_right_velocity_pid_params);
  up_down.set_velocity_controller(up_down_velocity_pid_params);

  //   Set closed-loop velocity mode
  left_right.set_control_mode(MotorGo::ControlMode::Velocity);
  up_down.set_control_mode(MotorGo::ControlMode::Velocity);

  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  // Start the WebSocket server
  //   webSocket.begin();
  //   webSocket.onEvent(webSocketEvent);

  //   Setup GCode parser
  //   Draw a 20 mm square
  //   std::vector<String> gcode_lines = {"G28",     "G90",         "G1 X50 Y50
  //   F50",
  //                                      "G91",     "G1 X20 F100", "G1 Y20",
  //                                      "G1 X-20", "G1 Y-20",     "G28"};

  stream = new GCode::WifiGCodeStream("192.168.0.15", 100);
  parser = new GCode::GCodeParser(stream, 990);

  GCode::start_parser(*parser);

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

  state.is_complete = false;

  //   profile_forward.x_initial = 0;
  //   profile_forward.y_initial = 0;
  //   profile_forward.x_final = 15;
  //   profile_forward.y_final = 0;
  //   profile_forward.v_initial = 0;
  //   profile_forward.v_final = 0;
  //   profile_forward.v_target = 30;
  //   profile_forward.a_target = ACCELERATION;

  //   profile_backward.x_initial = 15;
  //   profile_backward.y_initial = 0;
  //   profile_backward.x_final = 0;
  //   profile_backward.y_final = 0;
  //   profile_backward.v_initial = 0;
  //   profile_backward.v_final = 0;
  //   profile_backward.v_target = 30;
  //   profile_backward.a_target = ACCELERATION;

  //   Wait until button is pressed
  Serial.println("Ready to begin... Press button to start");
  while (true)
  {
    delay(100);
  }

  delay(1000);

  start_time = micros();

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

    left_right.set_target_velocity(left_right_velocity_target.load());
    up_down.set_target_velocity(up_down_velocity_target.load());

    left_right.loop();
    up_down.loop();

    esp_task_wdt_reset();
  }
}

void home()
{
  // Drive negative direction for 20 rads
  // Set target velocity to -5
  left_right.zero_position();
  while (left_right.get_position() > -30)
  {
    left_right_velocity_target.store(-5);
  }
  left_right_velocity_target.store(0);
  delay(10);

  left_right.zero_position();

  // Drive negative direction for 20 rads
  // Set target velocity to -5
  up_down.zero_position();
  while (up_down.get_position() > -20)
  {
    up_down_velocity_target.store(-5);
  }
  up_down_velocity_target.store(0);
  delay(10);

  up_down.zero_position();

  // Move 4 mm in the positive x and y direction away from the corner
  Planner::TrapezoidTrajectoryParameters zero;
  zero.x_initial = 0;
  zero.y_initial = 0;
  zero.x_final = 4;
  zero.y_final = 4;
  zero.v_initial = 0;
  zero.v_final = 0;
  zero.v_target = 50;
  zero.a_target = ACCELERATION / 2;

  Planner::TrapezoidVelocityTrajectory zero_trajectory =
      Planner::generate_trapezoid_profile(zero);

  Planner::TrajectoryState zero_state;
  zero_state.is_complete = false;
  while (!zero_state.is_complete)
  {
    long now = micros();
    zero_state =
        Planner::compute_trapezoid_velocity_vector(zero_trajectory, now);

    left_right_velocity_target.store(zero_state.v.x);
    up_down_velocity_target.store(zero_state.v.y);
  }

  left_right.zero_position();
  up_down.zero_position();
}

bool forward_or_backward = true;
Planner::TrapezoidVelocityTrajectory cur_trajectory;
unsigned long last_print_time = 0;
float last_final_x = 0;
float last_final_y = 0;
bool printed = false;
void loop()
{
  long now = micros();
  //   if (now - last_print_time > 1e6 / 30)
  //   {
  //     data_packet_t dataPacket;
  //     dataPacket.timestamp = now;
  //     dataPacket.sampleValue = state.v.x;
  //     strcpy(dataPacket.name, "command");

  //     webSocket.broadcastBIN(dataPacket.raw, DATA_PACKET_LEN);

  //     data_packet_t dataPacket2;
  //     dataPacket2.timestamp = now;
  //     dataPacket2.sampleValue = left_right.get_velocity();
  //     strcpy(dataPacket2.name, "actual");

  //     webSocket.broadcastBIN(dataPacket2.raw, DATA_PACKET_LEN);

  //     last_print_time = now;
  //   }
  // We've completed the previous motion AND there are more commands to
  // parse

  if (state.is_complete && parser->is_available())
  {
    // Retrieve next command
    GCode::MotionCommandResult result = parser->pop_command_buffer();
    // If the command was successfully retrieved
    if (result.success)
    {
      GCode::MotionCommand command = result.command;
      //   Ignore the rest of the command, run the homing procedure
      if (command.home)
      {
        // Skip for now, we'll implement this later
        home();
      }
      //   Otherwise, generate a trapezoid profile
      else
      {
        // delay(1000);
        Planner::TrapezoidTrajectoryParameters profile;
        profile.x_initial = last_final_x;
        profile.y_initial = last_final_y;
        profile.x_final = constrain(command.x * RAD_PER_MM, 0, X_LIM);
        profile.y_final = constrain(command.y * RAD_PER_MM, 0, Y_LIM);

        // For now, fix initial and final velocities to 0
        profile.v_initial = 0;
        profile.v_final = 0;
        profile.v_target = command.feedrate * MMPERMIN_TO_RADPERSEC;
        profile.a_target = ACCELERATION;

        // Serial.println("Command: " + String(command.x) + ", " +
        //                String(command.y) + ", " + String(command.feedrate));
        // // Print all profile parameters
        // Serial.println("Profile parameters:");
        // Serial.println("Initial position: " + String(profile.x_initial) + ",
        // " +
        //                String(profile.y_initial));
        // Serial.println("Final position: " + String(profile.x_final) + "," +
        //                String(profile.y_final));
        last_final_x = profile.x_final;
        last_final_y = profile.y_final;
        // Serial.println("Initial velocity: " + String(profile.v_initial));
        // Serial.println("Target velocity:" + String(profile.v_target));
        // Serial.println("Final velocity: " + String(profile.v_final));
        // Serial.println("Acceleration: " + String(profile.a_target));

        cur_trajectory = Planner::generate_trapezoid_profile(profile);

        // Serial.println("Accel time: " +
        //                String(cur_trajectory.acceleration_time_delta_us));
        // Serial.println("Coast time: " +
        //                String(cur_trajectory.coast_end_time_delta_us));
        // Serial.println("Decel time: " +
        //                String(cur_trajectory.end_time_delta_us));

        // Serial.println("End time: " +
        // String(cur_trajectory.end_time_delta_us));

        // Print times profile
        // Wait for the button press
        // while (digitalRead(0))
        // {
        //   delay(100);
        // }
        // delay(100);

        // Set the current time to the start time of the profile

        //   Only set complete to false if we have a new trajectory
        state.is_complete = false;
      }
    }
  }
  //   if (state.is_complete)
  //   {
  //     if (forward_or_backward)
  //     {
  //       cur_trajectory =
  //       Planner::generate_trapezoid_profile(profile_forward);
  //     }
  //     else
  //     {
  //       cur_trajectory =
  //       Planner::generate_trapezoid_profile(profile_backward);
  //     }
  //     forward_or_backward = !forward_or_backward;
  //     state.is_complete = false;
  //   }
  //   We're executing a command
  else
  {
    long now = micros();
    state = Planner::compute_trapezoid_velocity_vector(cur_trajectory, now);

    left_right_velocity_target.store(state.v.x);
    up_down_velocity_target.store(state.v.y);

    // Print is complete and is available on same line
    // Serial.println("Is complete: " + String(state.is_complete) +
    //                ", Is available: " + String(parser->is_available()));
  }

  if (parser->is_complete() && !printed)
  {
    // If we've completed all commands, print "DONE" and set printed to true
    Serial.println("DONE");
    Serial.println("Total Time: " + String((micros() - start_time) / 1e6) +
                   " s");
    left_right.disable();
    up_down.disable();
    printed = true;
  }
  //   left_right.loop();
  //   up_down.loop();
  //   if (enable_flag)
  //   {
  //     left_right.enable();
  //     up_down.enable();
  //     enable_flag = false;
  //     disable_flag = false;
  //   }
  //   else if (disable_flag)
  //   {
  //     left_right.disable();
  //     up_down.disable();
  //     disable_flag = false;
  //     enable_flag = false;
  //   }

  //   Print target velocity
  //   freq_println(
  //       "Target velocity: " + String(state.v.x - left_right.get_velocity()) +
  //           ", " + String(state.v.y - up_down.get_velocity()) + ", ",
  //       20);

  //   webSocket.loop();  // Handle WebSocket events
}
