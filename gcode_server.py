import socket

host = ""  # Listen on all interfaces
port = 5000

# Read in G-code file test/final.gcode
with open("../gcode-optimizer/test.gcode") as f:
    gcode_lines = f.readlines()
    gcode_lines.append("G28\n")

    # start_line = 0
    # end_line = 10

    # import ipdb

    # ipdb.set_trace()


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
