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

  cur_line_number = -max_buffer_len;
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

  //   Serial.println("Wifi GCode Stream initialized");
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
          //   Serial.println("Waiting for GCode server...");
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
      size_t write_data =
          client.print("GET /gcode?lines=" + String(max_buffer_len) +
                       "&start_line=" + String(cur_line_number) + " HTTP/1.1");

      //   Serial.println("Request sent for " + String(max_buffer_len) + "
      //   lines");
      if (write_data > 0)
      {
        // client.println();
        request_sent = true;
        request_start_time =
            millis();  // Record the time when the request was sent
      }
    }

    while (client.available() > 0 && next_buffer->len < max_buffer_len)
    {
      char c;

      client.readBytes(&c, 1);
      request_start_time =
          millis();  // Reset the timeout since data was received
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

    // Check for timeout
    if (request_sent)
    {
      unsigned long elapsed = millis() - request_start_time;
      if (elapsed > TIMEOUT_DURATION)
      {
        // Handle timeout
        Serial.println("Timeout waiting for response");
        client.stop();
        request_sent = false;
        next_buffer->len = 0;  // Clear the buffer to avoid partial data
        retry_count++;

        if (retry_count <= MAX_RETRIES)
        {
          // Retry sending the request
          Serial.println("Retrying request, attempt " + String(retry_count));
          // Reconnect to the server
          if (!client.connect(server_ip.c_str(), socket_port))
          {
            Serial.println("Failed to reconnect to server");
            // Optional: Implement reconnection attempts or delays here
          }
          else
          {
            // Reset retry count and timeout counter
            retry_count = 0;
            request_start_time = millis();
          }
        }
        else
        {
          // Maximum retries reached, handle accordingly
          Serial.println("Maximum retries reached. Aborting.");
          // Set appropriate flags or take necessary actions
          gcode_finished = true;  // Stop further processing
          retry_count = 0;        // Reset retry count for future operations

          // Optional: Implement further error handling here
        }
      }
    }

    // Check if the next buffer is ready
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

    // Reset next buffer and increment the line number to get the next batch
    cur_line_number += max_buffer_len;
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
  String address = "http://" + String(HOST) + ":" + String(PORT) +
                   "/gcode/available" + "?name=Claude-Monetch";

  http.begin(address.c_str());

  int http_code = http.GET();
  //   Serial.println("Server response: " + String(http_code));
  if (http_code != 200)
  {
    // Serial.println("Server not ready");
    return false;
  }

  http.end();

  return true;
}

void GCode::WifiGCodeStream::set_ready_for_next_gcode()
{
  //   ready_for_next_gcode = true;
}