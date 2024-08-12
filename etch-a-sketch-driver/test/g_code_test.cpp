#include <Arduino.h>
#include <unity.h>

#include <vector>

#include "configurable.h"
#include "gcode.h"
#include "motorgo_mini.h"

void gcode_parser_test()
{
  std::vector<String> lines = {
      "G90", "G0 X10 Y20 Z30 F100", "G1 X20 Y30 Z40 F200",
      "G91", "G1 X30 Y40 Z50 F300", "G1 X40 Y50 Z60 F400",
      "G28",
  };

  std::vector<GCode::MotionCommand> expected_commands;
  expected_commands.push_back({10, 20, 30, 100, false});
  expected_commands.push_back({20, 30, 40, 200, false});
  expected_commands.push_back({50, 70, 90, 300, false});
  expected_commands.push_back({90, 120, 150, 400, false});
  expected_commands.push_back({0, 0, 0, 0, true});

  GCode::VectorGCodeStream* stream = new GCode::VectorGCodeStream(lines);
  GCode::GCodeParser* parser = new GCode::GCodeParser(stream);

  GCode::start_parser(*parser);
  int i = 0;
  while (!parser->is_complete())
  {
    // Print buffer size and i
    if (parser->is_available())
    {
      GCode::MotionCommandResult result = parser->pop_command_buffer();
      if (result.success)
      {
        GCode::MotionCommand command = result.command;
        Serial.println("Command: " + String(command.x) + ", " +
                       String(command.y) + ", " + String(command.z) + ", " +
                       String(command.feedrate) + ", " + String(command.home));

        GCode::MotionCommand expected_command = expected_commands[i];
        if (!command.home)
        {
          TEST_ASSERT_EQUAL(expected_command.x, command.x);
          TEST_ASSERT_EQUAL(expected_command.y, command.y);
          TEST_ASSERT_EQUAL(expected_command.z, command.z);
          TEST_ASSERT_EQUAL(expected_command.feedrate, command.feedrate);
        }
        TEST_ASSERT_EQUAL(expected_command.home, command.home);
        i++;
      }
    }

    delay(1000);
  }
  Serial.println(i);
  TEST_ASSERT_EQUAL(expected_commands.size(), i);

  Serial.println("DONE");
}

void setup()
{
  Serial.begin(115200);
  delay(6000);
  UNITY_BEGIN();
  RUN_TEST(gcode_parser_test);
  UNITY_END();
}

void loop()
{
  // Nothing to do here
}
