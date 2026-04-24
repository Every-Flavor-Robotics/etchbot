#ifndef SERIAL_GCODE_STREAM_H
#define SERIAL_GCODE_STREAM_H

#include "gcode_stream.h"

namespace GCode
{

struct GCodeBuffer
{
    String lines[500];  // Fixed-size array to avoid heap fragmentation
    size_t current_line;
    size_t len;
};

class SerialGCodeStream : public GCodeStream
{
   public:
    SerialGCodeStream(int max_buffer_len = 500);

    void init() override;
    String read_line() override;
    void loop() override;
    bool is_complete() override;
    void set_ready_for_next_gcode() override;

   private:
    size_t max_buffer_len;
    bool gcode_finished;

    GCodeBuffer buffer_a;
    GCodeBuffer buffer_b;
    GCodeBuffer* cur_buffer;
    GCodeBuffer* next_buffer;
    bool next_buffer_ready;
    String next_line;

    bool gcode_started;  // Whether we've received GCODE_READY
};

}  // namespace GCode

#endif  // SERIAL_GCODE_STREAM_H
