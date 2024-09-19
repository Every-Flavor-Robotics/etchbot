#include <HTTPClient.h>
#include <WiFi.h>

#include "gcode_stream.h"

namespace GCode
{

struct GCodeBuffer
{
  std::vector<String> lines;
  size_t current_line;
  size_t len;
};

class WifiGCodeStream : public GCodeStream
{
 public:
  // Constructor
  WifiGCodeStream(String server_ip, int max_buffer_len = 500);

  void init() override;

  // Read a line from the stream
  String read_line() override;

  void loop() override;

  bool is_complete() override;

  void set_ready_for_next_gcode() override;

 private:
  const String server_ip;
  const int socket_port = 5005;
  WiFiClient client;
  size_t max_buffer_len;

  bool gcode_finished;

  GCodeBuffer* cur_buffer;
  GCodeBuffer* next_buffer;
  bool next_buffer_ready;
  String next_line;

  HTTPClient http;
  const int rest_port = STREAM_PORT;

  bool server_ready();
  bool ready_for_next_gcode = false;
  bool request_sent = false;
};
}  // namespace GCode