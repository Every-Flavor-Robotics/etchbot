#ifndef GCODE_STREAM_H
#define GCODE_STREAM_H

#include <Arduino.h>

#include <vector>

namespace GCode
{

// Define an abstract class GCodeStream
class GCodeStream
{
 public:
  // Constructor
  GCodeStream() {}

  // Destructor
  virtual ~GCodeStream() {}

  virtual void init() = 0;

  virtual bool is_complete() = 0;

  virtual void loop() = 0;

  // Read a line from the stream
  virtual String read_line() = 0;
};

class VectorGCodeStream : public GCodeStream
{
 public:
  // Constructor
  VectorGCodeStream(std::vector<String> lines);

  void init() override;

  // Read a line from the stream
  String read_line() override;

  bool is_complete() override;

 private:
  std::vector<String> lines;
  size_t current_line;
};
}  // namespace GCode

#endif  // GCODE_STREAM_H