#include "wifi_gcode_stream.h"

GCode::WifiGCodeStream::WifiGCodeStream(String server_ip, int max_buffer_len)
    : server_ip(server_ip),
      client(),
      max_buffer_len(max_buffer_len),
      gcode_finished(false),
      cur_buffer(new GCodeBuffer(
          {std::vector<String>(), 0, 0})),  // {lines, current_line, len}
      next_buffer(new GCodeBuffer(
          {std::vector<String>(), 0, 0})),  // {lines, current_line, len}
      next_buffer_ready(true),
      next_line("")
{
  // Initialize the buffers
  cur_buffer->lines.reserve(max_buffer_len);
  next_buffer->lines.reserve(max_buffer_len);
  for (int i = 0; i < max_buffer_len; i++)
  {
    cur_buffer->lines.push_back("");
    next_buffer->lines.push_back("");
  }
}

void GCode::WifiGCodeStream::init()
{
  // Initialize the GCode stream
  gcode_finished = true;
  cur_buffer->current_line = 0;
  cur_buffer->len = 0;
  next_buffer->current_line = 0;
  next_buffer->len = 0;

  ready_for_next_gcode = true;
  request_sent = false;

  Serial.println("Wifi GCode Stream initialized");
}

String GCode::WifiGCodeStream::read_line()
{
  if (cur_buffer->current_line >= cur_buffer->len)
  {
    return "";
  }

  cur_buffer->current_line++;
  return cur_buffer->lines[cur_buffer->current_line - 1];
}

void GCode::WifiGCodeStream::loop()
{
  // Waiting for new GCode to be ready
  if (gcode_finished)
  {
    if (ready_for_next_gcode && cur_buffer->current_line >= cur_buffer->len)
    {
      // If the server is ready, set gcode_finished to false
      gcode_finished = !server_ready();

      // If the server is ready, connect to the socket
      if (!gcode_finished)
      {
        // Serial.println("Server ready, connecting to GCode server");
        while (!client.connect(server_ip.c_str(), socket_port))
        {
          Serial.println("Waiting for GCode server...");
          delay(500);
        }
        //   Reset flag to fetch new GCode
        ready_for_next_gcode = false;
        gcode_finished = false;
      }
      else
      {
        // Give some time back
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }
  }
  else
  {  // Check if cur_buffer is all read
    //   If next buffer is ready, swap
    if (next_buffer->len == 0 && !request_sent)
    {
      // Send a request to the server for the next buffer
      client.print("GET /gcode?lines=" + String(max_buffer_len) + " HTTP/1.1");
      //   Serial.println("Request sent for " + String(max_buffer_len) + "
      //   lines");
      request_sent = true;
    }

    while (client.available() > 0 && next_buffer->len < max_buffer_len)
    {
      char c;

      client.readBytes(&c, 1);
      // If not a new line, we're not done reading the line
      //  Add the character to the next line and continue
      if (c != '\n')
      {
        next_line += c;
        continue;
      }

      // Get rid of white space
      next_line.trim();
      if (next_line == "")
      {
        // Serial.println("Skipping empty line");
        continue;
      }

      // We've reached the end of this GCode file, stop reading
      if (next_line == "END")
      {
        gcode_finished = true;
        //  Reset flag for new gcode fetch
        // This should only be set once the robot has finished executing the
        // current GCode file
        ready_for_next_gcode = false;

        // Disconnect from the server
        client.stop();
        // Serial.println("GCode file finished");

        break;
      }

      //
      // Serial.println("Adding line to next buffer: " + next_line);
      next_buffer->lines[next_buffer->len] = next_line;
      next_buffer->len++;
      next_line = "";
      request_sent = false;
    }

    //   Check if the next buffer is ready
    //   Either the buffer is full or the GCode file is finished
    if ((next_buffer->len == max_buffer_len || gcode_finished) &&
        !next_buffer_ready)
    {
      //   Serial.println("Next buffer ready");
      next_buffer_ready = true;
    }
  }

  if (cur_buffer->current_line >= cur_buffer->len && next_buffer_ready)
  {
    // Reset line number
    next_buffer->current_line = 0;

    GCodeBuffer *temp = cur_buffer;
    cur_buffer = next_buffer;

    // Reset next buffer
    next_buffer = temp;
    next_buffer_ready = false;
    next_buffer->len = 0;
  }
}

bool GCode::WifiGCodeStream::is_complete()
{
  // WiFi GCode stream is never complete, continuously checks for new GCode
  return false;
}

bool GCode::WifiGCodeStream::server_ready()
{
  // Check if the server is ready
  String address = "http://" + String(server_ip) + ":" + String(rest_port) +
                   "/gcode/available";

  http.begin(address.c_str());

  int http_code = http.GET();
  Serial.println("Server response: " + String(http_code));
  if (http_code != 200)
  {
    // Serial.println("Server not ready");
    return false;
  }

  return true;
}

void GCode::WifiGCodeStream::set_ready_for_next_gcode()
{
  //   ready_for_next_gcode = true;
}