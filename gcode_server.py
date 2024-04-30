import socket

host = ""  # Listen on all interfaces
port = 5000

# gcode_drawing = "pipeline_output/couch_hand_drawing/filtered.gcode"
# gcode_calibration = "pipeline_output/test_circle.gcode"

# gcode_lines = []
# with open(gcode_calibration) as f:
#     gcode_lines = f.readlines()
#     gcode_lines.append("G28\n")


title_cards = [
    "pipeline_output/title_10/filtered.gcode",
    "pipeline_output/title_09/filtered.gcode",
    "pipeline_output/title_08/filtered.gcode",
    "pipeline_output/title_07/filtered.gcode",
    "pipeline_output/title_06/filtered.gcode",
    "pipeline_output/title_05/filtered.gcode",
    "pipeline_output/mogodemo/filtered.gcode",
    "pipeline_output/pikachu/filtered.gcode",
    "pipeline_output/couch_hand_drawing/filtered.gcode",
    "pipeline_output/this_is_fine/filtered.gcode",
    "pipeline_output/doggo/filtered.gcode",
]

# Read all in and chain them together
gcode_lines = []
for title_card in title_cards:
    print("Reading title card:", title_card)
    with open(title_card) as f:
        gcode_lines.extend(f.readlines())
        gcode_lines.append("G28\n")

# Remove all comments
# for i in range(len(gcode_lines)):
#     # Look for a semicolon in line
#     comment_index = gcode_lines[i].find(";")
#     if comment_index != -1:
#         gcode_lines[i] = gcode_lines[i][:comment_index] + "\n"

# # Remove all empty lines
# gcode_lines = [line for line in gcode_lines if line.strip() != ""]


def main():
    start_line = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((host, port))
            s.listen()
            print("Server listening...")
            conn, addr = s.accept()
            with conn:
                print("Connected by", addr)
                while True:
                    data = conn.recv(1024).decode()
                    print("Received data")
                    print("------ data start ------")
                    print(data)
                    print("------ data end --------")
                    if data.startswith("GET /gcode?lines="):
                        # Extract the requested number of lines
                        try:
                            num_lines_requested = int(data.split("=")[1].split(" ")[0])
                        except (ValueError, IndexError):
                            conn.sendall(b"Invalid request format.\n")
                            continue

                        # Send the G-code lines
                        print(
                            len(
                                gcode_lines[
                                    start_line : start_line + num_lines_requested
                                ]
                            )
                        )
                        gcode_block = "".join(
                            gcode_lines[start_line : start_line + num_lines_requested]
                        )
                        start_line += num_lines_requested

                        if start_line >= len(gcode_lines):
                            start_line = 0
                        print("Sending response")
                        print(gcode_block)
                        conn.sendall(gcode_block.encode())
                        print("Response sent")
                    # else:
                    #     conn.sendall(b"Invalid request.\n")
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Closing socket...")
            s.close()


if __name__ == "__main__":
    main()


# import socketserver
# import time
# import threading

# kill_signal = False
# kill_signal_served = True


# class GCodeRequestHandler(socketserver.StreamRequestHandler):
#     start_line = 0

#     def handle(self):
#         global kill_signal, kill_signal_served

#         print("Connected by", self.client_address)

#         # Signal to all previous processes to stop
#         kill_signal = True
#         while not kill_signal_served:
#             time.sleep(1)
#         # Reset the kill signal
#         kill_signal_served = False
#         kill_signal = False

#         last_heartbeat = time.time()
#         # Start the heartbeat thread
#         threading.Thread(target=self.send_heartbeat, args=(1,)).start()

#         while not kill_signal:
#             print("Waiting for data...")
#             data = self.request.recv(1024).strip().decode()
#             print(type(data))
#             print("Received data")
#             print("------ data start ------")
#             print(data)
#             print("------ data end --------")
#             if data.startswith("GET /gcode?lines="):
#                 try:
#                     num_lines_requested = int(data.split("=")[1].split(" ")[0])
#                 except (ValueError, IndexError):
#                     self.wfile.write(b"Invalid request format.\n")
#                     continue
#                 # Send the G-code lines
#                 print(
#                     len(
#                         gcode_lines[
#                             self.start_line : self.start_line + num_lines_requested
#                         ]
#                     )
#                 )
#                 gcode_block = "".join(
#                     gcode_lines[self.start_line : self.start_line + num_lines_requested]
#                 )
#                 self.start_line += num_lines_requested
#                 if self.start_line >= len(gcode_lines):
#                     self.start_line = 0
#                 print("Sending response")
#                 print(gcode_block)
#                 self.wfile.write(gcode_block.encode())
#                 print("Response sent")

#             # if time.time() - last_heartbeat > 1:
#             #     self.send_heartbeat(1)
#             #     last_heartbeat = time.time()
#         print("Received a kill signal... Closing connection.")

#     def send_heartbeat(self, interval):

#         last_heartbeat = time.time()
#         while True:
#             if time.time() - last_heartbeat > interval:
#                 # Heartbeat is 0x00 ASCII
#                 print("Sending heartbeat...")
#                 heartbeat_message = "\x01"
#                 self.request.sendall(heartbeat_message.encode())
#                 last_heartbeat = time.time()


# if __name__ == "__main__":
#     host, port = "0.0.0.0", 5000  # Your host and port here
#     with socketserver.TCPServer((host, port), GCodeRequestHandler) as server:
#         print("Server listening...")
#         server.serve_forever()
