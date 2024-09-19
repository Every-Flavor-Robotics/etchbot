#ifndef GCODE_H
#define GCODE_H

#include <Arduino.h>

#include <atomic>
#include <queue>
#include <vector>

#include "gcode_stream.h"

namespace GCode
{

enum class G0G1Mode
{
  G0,
  G1
};

enum class G2G3Mode
{
  G2,
  G3
};

enum class G90G91Mode
{
  G90,
  G91
};

struct MotionCommand
{
  float x;
  float y;
  float z;
  float feedrate;
  bool home;

  MotionCommand() : x(0), y(0), z(0), feedrate(0), home(false) {}

  MotionCommand(float x, float y, float z, float feedrate, bool home)
      : x(x), y(y), z(z), feedrate(feedrate), home(home)
  {
  }
};

struct MotionCommandResult
{
  MotionCommand command;
  bool success;
};

struct GCodeParameter
{
  String name;  // e.g., "X", "Y", "Z"
  float value;
};

class GCodeParser
{
 public:
  // Constructor (you might want to initialize variables here)
  GCodeParser(GCodeStream *stream, size_t max_buffer_size = 10,
              float rapid_feedrate = 12000.0f);

  void init();

  // Primary parsing function
  void parse_line(String line);
  String get_next_gcode_line();
  void loop();
  bool is_complete();
  bool is_stream_complete();
  bool is_available();

  size_t buffer_size;
  size_t max_buffer_size;

  int get_command_buffer_size();
  void push_command_buffer(const MotionCommand &command);
  MotionCommandResult peek_command_buffer();
  MotionCommandResult pop_command_buffer();

  void set_ready_for_next_gcode();

 private:
  std::queue<MotionCommand> motion_command_buffer;
  std::atomic<bool> lock_command_buffer;

  bool acquire_lock();
  bool acquire_lock_no_timeout();

  void release_lock();

  // Internal state variables
  double current_x, current_y, current_z;  // Current machine position
  bool absolute_mode;                      // G90 (absolute) or G91 (relative)
  float feedrate;
  float rapid_feedrate;
  GCodeStream *stream;

  // Helper functions
  void process_g0_g1(const String &line, G0G1Mode mode);  // Linear movement
  void process_g2_g3(const String &line,
                     G2G3Mode mode);     // Circular/helical movement
  void process_g28(const String &line);  // Homing
  void process_g90_g91(const String &line,
                       G90G91Mode);  // Absolute/relative mode
  std::vector<GCodeParameter> extract_parameters(const String &line);

  void set_feedrate(float feedrate);
  void set_rapid_feedrate(float rapid_feedrate);
  void set_absolute_positioning();
  void set_relative_positioning();

  float get_feedrate();
  float get_rapid_feedrate();
};

void parser_thread(GCodeParser &parser);
void start_parser(GCodeParser &parser);

}  // namespace GCode
#endif  // GCODE_H