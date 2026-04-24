#include <Arduino.h>
#include <motorgo_mini.h>

#include "motor_definitions.h"

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

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& left_right = motorgo_mini.ch1;
MotorGo::MotorChannel& up_down = motorgo_mini.ch0;

MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

uint8_t erase_out_pin = QWIIC_SDA;
uint8_t erase_in_pin = QWIIC_SCL;

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
  float inset = 8;

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

  // Move past the zero point by 2 mm
  target = RAD_PER_MM * (inset + LEFT_RIGHT_BACKLASH_MM + 2.);
  // Move right
  left_right.set_target_velocity(5.0 * homing_speed);
  while (left_right.get_position() < target)
  {
    left_right.loop();
  }
  left_right.set_target_velocity(0.0);
  left_right.disable();
  left_right.loop();

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
  up_down.set_target_velocity(5.0 * homing_speed);
  while (up_down.get_position() < target)
  {
    up_down.loop();
  }
  up_down.set_target_velocity(0.0);
  up_down.disable();
  up_down.loop();

  // Move left quickly towards the home
  target = RAD_PER_MM * inset + 0.5;
  left_right.enable();
  left_right.set_target_velocity(-1.5 * homing_speed);
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

  // Move down quickly towards the home
  target = RAD_PER_MM * inset + 0.5;
  up_down.enable();
  up_down.set_target_velocity(-1.5 * homing_speed);
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

void setup()
{
  pinMode(erase_out_pin, OUTPUT);
  pinMode(erase_in_pin, INPUT);

  digitalWrite(erase_out_pin, LOW);

  // Setup motor parameters
  config_ch0.motor_config = GARTTLeftEtch1;
  config_ch0.power_supply_voltage = 15.0;
  config_ch0.reversed = false;

  config_ch1.motor_config = GARTTRightEtch1;
  config_ch1.power_supply_voltage = 15.0;
  config_ch1.reversed = false;

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  //   state.is_complete = false;
  left_right.loop();
  up_down.loop();

  left_right.zero_position();
  up_down.zero_position();

  home();
}

void loop() { delay(1000); }
