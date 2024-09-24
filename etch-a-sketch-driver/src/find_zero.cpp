#include <Arduino.h>
#include <motorgo_mini.h>

#include "motor_definitions.h"

uint8_t erase_out_pin = QWIIC_SDA;
uint8_t erase_in_pin = QWIIC_SCL;

//  Conversion from mm to radians
// #define GEAR_RATIO (18.0f / 81.0f)  // Motor to knob
#define RAD_PER_MM (MM_TO_KNOB_RAD / GEAR_RATIO)
#define MMPERMIN_TO_RADPERSEC (RAD_PER_MM / 60.0)

#define UP_DOWN_BACKLASH_RAD (UP_DOWN_BACKLASH_MM * RAD_PER_MM)
#define LEFT_RIGHT_BACKLASH_RAD (LEFT_RIGHT_BACKLASH_MM * RAD_PER_MM)
// mm * RAD_PER_MM = rad

#define X_LIM 130
#define Y_LIM 89.375

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& left_right = motorgo_mini.ch1;
MotorGo::MotorChannel& up_down = motorgo_mini.ch0;

MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

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

  bool calibrate = false;
  left_right.init(config_ch0, calibrate);
  up_down.init(config_ch1, calibrate);

  left_right.set_control_mode(MotorGo::ControlMode::Voltage);
  up_down.set_control_mode(MotorGo::ControlMode::Voltage);
}

float normalize_angle(float angle)
{
  // Get angle in the range of -Pi/2 to 3Pi/2
  while (angle < -PI / 2)
  {
    angle += TWO_PI;
  }
  while (angle > 3 * PI / 2)
  {
    angle -= TWO_PI;
  }
  return angle;
}

void loop()
{
  left_right.loop();
  up_down.loop();

  float left_right_position = normalize_angle(left_right.get_position());
  float up_down_position = normalize_angle(up_down.get_position());

  float left_right_position_mm = left_right_position / RAD_PER_MM;
  float up_down_position_mm = up_down_position / RAD_PER_MM;

  String str = "Position (mm): " + String(left_right_position_mm, 4) + ", " +
               String(up_down_position_mm, 4);
  str += "\tPosition (rad): " + String(left_right_position, 4) + ", " +
         String(up_down_position, 4);

  freq_println(str, 30);
}
