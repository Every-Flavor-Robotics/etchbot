#include "gcode.h"

#include <esp_task_wdt.h>

#include <chrono>

GCode::GCodeParser::GCodeParser(GCodeStream* stream, size_t max_buffer_size,
                                float rapid_feedrate)
    : buffer_size(max_buffer_size),
      max_buffer_size(max_buffer_size),
      current_x(0.0),
      current_y(0.0),
      current_z(0.0),
      absolute_mode(true),
      feedrate(0.0),
      rapid_feedrate(rapid_feedrate),
      stream(stream)
{
  lock_command_buffer.store(false);
}

void GCode::GCodeParser::parse_line(String line)
{
  // Basic Preprocessing
  line.trim();         // Remove leading/trailing whitespace
  line.toUpperCase();  // Convert to uppercase for easier parsing

  // Extract G-Code and parameters
  int gCodeIndex = line.indexOf('G');
  int spaceIndex = line.indexOf(' ');

  if (gCodeIndex != -1)
  {
    String gCodeStr;

    // Extract G-code
    if (spaceIndex != -1)
    {
      gCodeStr = line.substring(gCodeIndex + 1, spaceIndex);
    }
    else
    {
      gCodeStr = line.substring(gCodeIndex + 1);
    }

    int gCode = gCodeStr.toInt();

    // Dispatch to helper functions based on G-Code
    switch (gCode)
    {
      case 0:
        process_g0_g1(line, GCode::G0G1Mode::G0);
        // Serial.println("Running G0");
        break;
      case 1:
        process_g0_g1(line, GCode::G0G1Mode::G1);
        // Serial.println("Running G1");

        break;
      case 2:  // Falls through to G3
      case 3:
        // process_g2_g3(line);
        Serial.println("G2/G3 not implemented yet");
        break;
      case 28:
        process_g28(line);
        // Serial.println("Running G28");

        break;
      case 90:
        process_g90_g91(line, GCode::G90G91Mode::G90);
        // Serial.println("Running G90");

        break;
      case 91:
        process_g90_g91(line, GCode::G90G91Mode::G91);
        // Serial.println("Running G91");

        break;
      default:
        //   For now, let's ignore unknown G-codes
        // Give a warning message
        // Serial.println("Skipping unknown G-code: " + gCodeStr);
        break;
    }
  }
  else
  {
    // For now, let's ignore lines without G-codes
  }
}

void GCode::GCodeParser::process_g0_g1(const String& line, GCode::G0G1Mode mode)
{
  // Construct the motion command
  MotionCommand motion_command;

  if (absolute_mode)
  {
    // If absolute mode, set coordinate frame to origin
    motion_command.x = 0.0;
    motion_command.y = 0.0;
    motion_command.z = 0.0;
  }
  else
  {
    // If relative mode, set coordinate frame to current position
    motion_command.x = current_x;
    motion_command.y = current_y;
    motion_command.z = current_z;
  }

  // Extract Parameters
  std::vector<GCode::GCodeParameter> params = extract_parameters(line);

  // Update Position Based on Parameters
  for (const GCode::GCodeParameter& param : params)
  {
    if (param.name == "X")
    {
      motion_command.x += param.value;
    }
    else if (param.name == "Y")
    {
      motion_command.y += param.value;
    }
    else if (param.name == "Z")
    {
      motion_command.z += param.value;
    }
    else if (param.name == "F")
    {
      set_feedrate(param.value);
    }
  }

  // Update Current Position
  current_x = motion_command.x;
  current_y = motion_command.y;
  current_z = motion_command.z;

  if (mode == G0G1Mode::G1)
  {
    // Set feedrate for G1 moves
    motion_command.feedrate = get_feedrate();
  }
  else
  {
    // Set feedrate for G0 moves
    motion_command.feedrate = get_rapid_feedrate();
  }

  // Add to Command Buffer
  push_command_buffer(motion_command);
}

void GCode::GCodeParser::process_g28(const String& line)
{
  // Home the machine
  MotionCommand motion_command;
  motion_command.home = true;

  // Add to Command Buffer
  push_command_buffer(motion_command);
}

void GCode::GCodeParser::process_g90_g91(const String& line,
                                         GCode::G90G91Mode mode)
{
  if (mode == G90G91Mode::G90)
  {
    set_absolute_positioning();
  }
  else
  {
    set_relative_positioning();
  }
}

std::vector<GCode::GCodeParameter> GCode::GCodeParser::extract_parameters(
    const String& line)
{
  // Line will look like:
  //   G1 X10.2 Y20 Z30 F100
  std::vector<GCodeParameter> parameters;

  //   Serial.println("Extracting parameters from line: " + line);

  // Find first space, which separates G-code from parameters
  int current_index = line.indexOf(' ');
  // Loop over all parameters
  while (current_index < line.length() && current_index != -1)
  {
    // Find next parameter based on next space
    int param_end_index = line.indexOf(' ', current_index + 1);

    // If no space found, assume end of line
    if (param_end_index == -1)
    {
      param_end_index = line.length();
    }

    // Extract parameter
    String param_str = line.substring(current_index + 1, param_end_index);

    // Assume parameter is of the form "X10.2"
    GCodeParameter param;
    param.name = param_str.substring(0, 1);
    // Everything from 1 to end is the value
    param.value = param_str.substring(1).toFloat();

    // Add to vector
    parameters.push_back(param);

    // Move to next parameter
    current_index = param_end_index;
  }

  return parameters;
}

float GCode::GCodeParser::get_feedrate() { return feedrate; }
float GCode::GCodeParser::get_rapid_feedrate() { return rapid_feedrate; }

void GCode::parser_thread(GCode::GCodeParser& parser)
{
  while (!parser.is_stream_complete())
  {
    if (parser.get_command_buffer_size() < parser.buffer_size)
    {
      // Get a line of GCode
      String gcode_line = parser.get_next_gcode_line();

      // Parse Line
      parser.parse_line(gcode_line);
    }
    else
    {
      // Brief pause if buffer is full
      vTaskDelay(2);
    }

    parser.loop();

    // Check if the stream is completeq
    if (parser.get_command_buffer_size() == 0)
    {
      parser.set_ready_for_next_gcode();
    }

    // Reset the watchdog timer
    esp_task_wdt_reset();
  }

  Serial.println("G-code parsing complete");
  vTaskDelete(NULL);
}

void GCode::start_parser(GCode::GCodeParser& parser)
{
  parser.init();

  // Create a lambda function that calls parser_thread with the parser object
  auto parser_thread_wrapper = [](void* param)
  {
    GCode::GCodeParser* parser = static_cast<GCode::GCodeParser*>(param);
    GCode::parser_thread(*parser);
  };

  xTaskCreate(parser_thread_wrapper, "GCodeParser", 4096, &parser, 0, NULL);
}

void GCode::GCodeParser::init()
{
  // Initialize the stream
  stream->init();
}

String GCode::GCodeParser::get_next_gcode_line() { return stream->read_line(); }

void GCode::GCodeParser::loop() { stream->loop(); }

void GCode::GCodeParser::set_feedrate(float feedrate)
{
  this->feedrate = feedrate;
}

void GCode::GCodeParser::set_rapid_feedrate(float rapid_feedrate)
{
  this->rapid_feedrate = rapid_feedrate;
}

void GCode::GCodeParser::set_absolute_positioning() { absolute_mode = true; }

void GCode::GCodeParser::set_relative_positioning() { absolute_mode = false; }

bool GCode::GCodeParser::is_complete()
{
  return stream->is_complete() && get_command_buffer_size() == 0;
}

bool GCode::GCodeParser::is_stream_complete() { return stream->is_complete(); }

void GCode::GCodeParser::set_ready_for_next_gcode()
{
  stream->set_ready_for_next_gcode();
}

bool GCode::GCodeParser::is_available()
{
  return get_command_buffer_size() > 0;
}

// TODO: Potential edge cases for returning -1
int GCode::GCodeParser::get_command_buffer_size()
{
  if (!acquire_lock())
  {
    return -1;
  }
  int size = motion_command_buffer.size();
  release_lock();
  return size;
}

void GCode::GCodeParser::push_command_buffer(const MotionCommand& command)
{
  if (!acquire_lock())
  {
    return;
  }
  motion_command_buffer.push(command);
  release_lock();
}

GCode::MotionCommandResult GCode::GCodeParser::peek_command_buffer()
{
  MotionCommandResult result;

  if (!acquire_lock())
  {
    result.success = false;
    return result;
  }

  if (motion_command_buffer.empty())
  {
    result.success = false;
  }
  else
  {
    result.command = motion_command_buffer.front();
    result.success = true;
  }

  release_lock();
  return result;
}

GCode::MotionCommandResult GCode::GCodeParser::pop_command_buffer()
{
  MotionCommandResult result;

  if (!acquire_lock_no_timeout())
  {
    result.success = false;
    return result;
  }

  if (motion_command_buffer.empty())
  {
    result.success = false;
  }
  else
  {
    result.command = motion_command_buffer.front();
    motion_command_buffer.pop();
    result.success = true;
  }

  release_lock();

  return result;
}

bool GCode::GCodeParser::acquire_lock()
{
  bool expected = false;
  bool acquired = lock_command_buffer.compare_exchange_strong(expected, true);

  // Try for a timeout period to acquire the lock
  int timeout = 10;  // Timeout in milliseconds
  int elapsed_time = 0;
  while (!acquired && elapsed_time < timeout)
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    elapsed_time += 10;
    acquired = lock_command_buffer.compare_exchange_strong(expected, true);
  }

  return acquired;
}

bool GCode::GCodeParser::acquire_lock_no_timeout()
{
  bool expected = false;

  return lock_command_buffer.compare_exchange_strong(expected, true);
}

void GCode::GCodeParser::release_lock() { lock_command_buffer.store(false); }