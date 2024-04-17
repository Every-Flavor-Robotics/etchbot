#include <Arduino.h>
#include <WebSocketsServer.h>  // Add this for WebSockets
#include <esp_task_wdt.h>
#include <motorgo_mini.h>

#include <atomic>
#include <chrono>
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
#define MMPERMIN_TO_RADPERSEC RAD_PER_MM / 60.0
#define ACCELERATION 1500

#define X_LIM 130 * RAD_PER_MM
#define Y_LIM 89.375 * RAD_PER_MM

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

  left_right_velocity_pid_params.p = 0.2;
  left_right_velocity_pid_params.i = 0.0;
  left_right_velocity_pid_params.d = 0.0;
  left_right_velocity_pid_params.lpf_time_constant = 0.0001;

  up_down_velocity_pid_params.p = 0.2;
  up_down_velocity_pid_params.i = 0.0;
  up_down_velocity_pid_params.d = 0.0;
  up_down_velocity_pid_params.lpf_time_constant = 0.0001;

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

  profile_forward.x_initial = 0;
  profile_forward.y_initial = 0;
  profile_forward.x_final = 15;
  profile_forward.y_final = 0;
  profile_forward.v_initial = 0;
  profile_forward.v_final = 0;
  profile_forward.v_target = 15;
  profile_forward.a_target = ACCELERATION;

  profile_backward.x_initial = 15;
  profile_backward.y_initial = 0;
  profile_backward.x_final = 0;
  profile_backward.y_final = 0;
  profile_backward.v_initial = 0;
  profile_backward.v_final = 0;
  profile_backward.v_target = 15;
  profile_backward.a_target = ACCELERATION;

  //   Wait until button is pressed
  Serial.println("Ready to begin... Press button to start");
  while (digitalRead(0))
  {
    delay(100);
  }

  delay(1000);

  start_time = micros();

  //   left_right.enable();
  //   up_down.enable();
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

    Serial.println("X: " + String(left_right.get_velocity(), 5));
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

bool forward_or_backward = true;
Planner::TrapezoidVelocityTrajectory cur_trajectory;
unsigned long last_print_time = 0;
float last_final_x = 0;
float last_final_y = 0;
bool printed = false;
// auto start = std::chrono::high_resolution_clock::now();
void loop()
{
  //   unsigned long now = micros();
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

  if (state.is_complete)
  {
    if (forward_or_backward)
    {
      cur_trajectory = Planner::generate_trapezoid_profile(profile_forward);
    }
    else
    {
      cur_trajectory = Planner::generate_trapezoid_profile(profile_backward);
    }
    forward_or_backward = !forward_or_backward;
    state.is_complete = false;
  }
  //   We're executing a command
  else
  {
    unsigned long now = micros();
    state = Planner::compute_trapezoid_velocity_vector(cur_trajectory, now);

    left_right_velocity_target.store(state.v.x);
    up_down_velocity_target.store(state.v.y);

    // Print is complete and is available on same line
    // Serial.println("Is complete: " + String(state.is_complete) +
    //                ", Is available: " + String(parser->is_available()));
  }

  //   webSocket.loop();  // Handle WebSocket events
}
