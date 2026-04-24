#include "serial_gcode_stream.h"

GCode::SerialGCodeStream::SerialGCodeStream(int max_buffer_len)
    : max_buffer_len(max_buffer_len),
      gcode_finished(true),
      next_buffer_ready(false),
      next_line(""),
      gcode_started(false)
{
    cur_buffer = &buffer_a;
    next_buffer = &buffer_b;

    cur_buffer->current_line = 0;
    cur_buffer->len = 0;
    next_buffer->current_line = 0;
    next_buffer->len = 0;
}

void GCode::SerialGCodeStream::init()
{
    gcode_finished = true;
    gcode_started = false;
    cur_buffer->current_line = 0;
    cur_buffer->len = 0;
    next_buffer->current_line = 0;
    next_buffer->len = 0;
    next_buffer_ready = false;
}

String GCode::SerialGCodeStream::read_line()
{
    if (cur_buffer->current_line >= cur_buffer->len)
    {
        return "";
    }

    cur_buffer->current_line++;
    return cur_buffer->lines[cur_buffer->current_line - 1];
}

void GCode::SerialGCodeStream::loop()
{
    // Read incoming serial data and assemble lines
    while (Serial.available() > 0 && next_buffer->len < max_buffer_len)
    {
        char c = Serial.read();

        if (c != '\n')
        {
            if (c != '\r')  // Ignore carriage returns
            {
                next_line += c;
            }
            continue;
        }

        // We have a complete line
        next_line.trim();
        if (next_line == "")
        {
            continue;
        }

        // Check for control messages
        if (next_line == "GCODE_READY")
        {
            gcode_started = true;
            gcode_finished = false;
            next_line = "";
            continue;
        }

        if (next_line == "END")
        {
            gcode_finished = true;
            gcode_started = false;
            next_line = "";
            // Mark next buffer as ready if it has any content
            if (next_buffer->len > 0)
            {
                next_buffer_ready = true;
            }
            break;
        }

        // Only buffer GCode lines if we've received GCODE_READY
        if (gcode_started)
        {
            next_buffer->lines[next_buffer->len] = next_line;
            next_buffer->len++;
            next_line = "";
        }
        else
        {
            // Non-GCode control messages are ignored here
            // (COMMAND:xxx is handled in main_stepper.cpp before draw_loop runs)
            next_line = "";
        }
    }

    // Check if next buffer is full
    if (next_buffer->len >= max_buffer_len && !next_buffer_ready)
    {
        next_buffer_ready = true;
    }

    // Swap buffers when cur_buffer is exhausted and next is ready
    if (cur_buffer->current_line >= cur_buffer->len && next_buffer_ready)
    {
        next_buffer->current_line = 0;

        GCodeBuffer* temp = cur_buffer;
        cur_buffer = next_buffer;
        next_buffer = temp;

        next_buffer_ready = false;
        next_buffer->len = 0;
    }
}

bool GCode::SerialGCodeStream::is_complete()
{
    return false;
}

void GCode::SerialGCodeStream::set_ready_for_next_gcode()
{
    // Could be used to signal readiness for next GCode file
}
