#include "gcode_stream.h"

GCode::VectorGCodeStream::VectorGCodeStream(std::vector<String> lines)
    : lines(lines)
{
}

void GCode::VectorGCodeStream::init() { current_line = 0; }

String GCode::VectorGCodeStream::read_line()
{
  if (current_line < lines.size())
  {
    current_line++;
    return lines[current_line - 1];
  }
  else
  {
    return "";
  }
}

bool GCode::VectorGCodeStream::is_complete()
{
  //   Serial.println("######");
  //   Serial.println(current_line);
  //   Serial.println(lines.size());
  //   Serial.println("######");

  return current_line >= lines.size();
}