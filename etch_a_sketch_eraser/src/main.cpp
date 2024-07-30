#include <Arduino.h>
#include <motorgo_mini.h>

#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "pid_manager.h"

#define FLIPPING_VELOCITY_RAD_PER_SEC 1.5

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_right = motorgo_mini.ch0;

MotorGo::MotorConfiguration gm3506_config;
MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

// Velocity controller parameters
MotorGo::PIDParameters velocity_controller_parameters_ch0;
MotorGo::PIDParameters velocity_controller_parameters_ch1;

// Position controller parameters
MotorGo::PIDParameters position_controller_parameters;
PIDController position_controller(position_controller_parameters.p,
                                  position_controller_parameters.i,
                                  position_controller_parameters.d,
                                  position_controller_parameters.output_ramp,
                                  position_controller_parameters.limit);

// configure wifi communications
bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    // motor_left.enable();
    motor_right.enable();
  }
  else
  {
    Serial.println("Disabling motors");
    // motor_left.disable();
    motor_right.disable();
  }
}

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
  Serial.begin(5000000);

  gm3506_config.pole_pairs = 11;
  gm3506_config.phase_resistance = 5.6;
  gm3506_config.calibration_voltage = 1.0;

  config_ch1.motor_config = gm3506_config;
  config_ch1.power_supply_voltage = 6.0;
  config_ch1.reversed = true;

  motor_right.init(config_ch1, false);

  velocity_controller_parameters_ch1.p = 1.0;
  velocity_controller_parameters_ch1.lpf_time_constant = 0.1;

  // Configure the position controller
  position_controller.P = 3.5;
  position_controller.I = 0.1;

  //   We do this ONLY to update the internal low-pass filter for the position
  //   output
  position_controller_parameters.lpf_time_constant = 0.1;
  motor_right.set_position_controller(position_controller_parameters);

  motor_right.set_velocity_controller(velocity_controller_parameters_ch1);
  motor_right.set_control_mode(MotorGo::ControlMode::Velocity);
  motor_right.set_velocity_limit(FLIPPING_VELOCITY_RAD_PER_SEC);
  motor_right.zero_position();

  enable_motors.set_post_callback(enable_motors_callback);

  pinMode(CH1_UH, OUTPUT);
  pinMode(CH1_UL, OUTPUT);
  pinMode(CH1_VH, OUTPUT);
  pinMode(CH1_VL, OUTPUT);

  digitalWrite(CH1_UH, LOW);
  digitalWrite(CH1_UL, LOW);
  digitalWrite(CH1_VH, LOW);
  digitalWrite(CH1_VL, LOW);

  ledcSetup(6, 20000, 8);
  ledcSetup(7, 20000, 8);

  ledcAttachPin(CH1_UL, 6);
  ledcAttachPin(CH1_VL, 7);

  motor_right.enable();
}

unsigned long last_time = micros();
unsigned long hold_time = 5 * 1e6;

float zero_position = 0.0;
float erase_position_1 = 1.62;
float erase_position_2 = 0.9;
float target_pos = erase_position_1;

int cur_cycle_count = 0;
int erase_cycles = 3;

bool current_pos = false;
int erase_motor_pwm = 0.0;
void loop()
{
  float error = target_pos - motor_right.get_position();
  float output = position_controller(error);
  motor_right.set_target_velocity(output);
  // Update the motors
  motor_right.loop();

  erase_motor_pwm = 255 * (2.0 / config_ch1.power_supply_voltage);

  digitalWrite(CH1_UH, HIGH);
  ledcWrite(7, erase_motor_pwm);

  if (cur_cycle_count >= erase_cycles)
  {
    Serial.println("Erasing complete");

    // Stop motors
    motor_right.disable();

    ledcWrite(7, 0);

    while (true)
    {
      delay(1000);
    }
  }

  if (micros() - last_time > hold_time)
  {
    if (current_pos)
    {
      target_pos = erase_position_1;
      current_pos = false;
    }
    else
    {
      target_pos = erase_position_2;
      current_pos = true;
      cur_cycle_count++;

      Serial.println("Cycle count: " + String(cur_cycle_count));
    }

    last_time = micros();
  }
}

// #include <Arduino.h>

// void setup()
// {
//   Serial.begin(5000000);

//   pinMode(CH1_UH, OUTPUT);
//   pinMode(CH1_UL, OUTPUT);
//   pinMode(CH1_VH, OUTPUT);
//   pinMode(CH1_VL, OUTPUT);

//   digitalWrite(CH1_UH, LOW);
//   digitalWrite(CH1_UL, LOW);
//   digitalWrite(CH1_VH, LOW);
//   digitalWrite(CH1_VL, LOW);

//   ledcSetup(6, 20000, 8);
//   ledcSetup(7, 20000, 8);

//   ledcAttachPin(CH1_UL, 6);
//   ledcAttachPin(CH1_VL, 7);
// }

// void loop()
// {
//   digitalWrite(CH1_UH, HIGH);
//   ledcWrite(7, 64);
// }
