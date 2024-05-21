import pathlib
import socketserver
import time
import click
import threading
import socket
from flask import Flask, jsonify

gcode_buffer = []
buffer_lock = threading.Lock()


class GCode:
    def __init__(self, gcode_file, lines=None):
        self.gcode_file = gcode_file

        if lines != None:
            self.gcode_lines = lines
        else:
            self.gcode_lines = []
            with open(gcode_file) as f:
                self.gcode_lines = f.readlines()
                self.gcode_lines.append("G28\n")
                self.gcode_lines.append("END\n")

        self.line_number = 0

    def get_gcode(self, num_lines):
        gcode_block = "".join(
            self.gcode_lines[self.line_number : self.line_number + num_lines]
        )
        self.line_number += num_lines
        return gcode_block

    def complete(self):
        return self.line_number >= len(self.gcode_lines)

    def delete_file(self):
        if self.gcode_file is not None:
            self.gcode_file.unlink()

    def reset(self):
        self.line_number = 0


class EmptyGCode(GCode):
    def __init__(self):
        # Call init with END line only
        super().__init__(None, ["END\n"])


class GCodeRequestHandler(socketserver.StreamRequestHandler):
    start_line = 0

    def handle(self):
        global gcode_buffer, buffer_lock

        print("Connected by", self.client_address)

        with buffer_lock:
            if len(gcode_buffer) == 0:
                print("No GCode in buffer. Sending empty GCode.")
                gcode = EmptyGCode()
            else:
                gcode = gcode_buffer.pop(0)

        try:
            while not gcode.complete():
                self.request.settimeout(5)

                print("Waiting for data...")
                data = self.request.recv(1024).strip().decode()
                if data.startswith("GET /gcode?lines="):
                    try:
                        num_lines_requested = int(data.split("=")[1].split(" ")[0])
                    except (ValueError, IndexError):
                        self.request.sendall(b"Invalid request format.\n")
                        continue
                    # Send the G-code lines
                    gcode_block = gcode.get_gcode(num_lines_requested)

                    print("Sending response")
                    self.request.sendall(gcode_block.encode())

                    print("Response sent")
            # Delete gcode file
            gcode.delete_file()
        except socket.timeout:
            print("Connection timed out. Will retry this gcode next time")
            # Put gcode back in the buffer
            gcode.reset()
            with buffer_lock:
                gcode_buffer.insert(0, gcode)

        except (ConnectionResetError, BrokenPipeError):
            print("Connection closed by client. Will retry this gcode next time")

            # Put gcode back in the buffer
            gcode.reset()
            with buffer_lock:
                gcode_buffer.insert(0, gcode)

        # Close the socket and connection
        print("G-code transmission ended. Closing connection.")
        self.request.close()


def get_new_gcode(scan_directory: pathlib.Path):
    """Thread for reading new GCode files"""
    global gcode_buffer, buffer_lock

    print("Scanning directory:", scan_directory)

    processed_files = set()
    while True:
        # Check for new GCode files
        new_gcode_files = list(scan_directory.glob("*.gcode"))

        # Remove already processed files from the list
        new_gcode_files = [
            gcode_file
            for gcode_file in new_gcode_files
            if gcode_file not in processed_files
        ]

        # Process new GCode files
        for gcode_file in new_gcode_files:
            print("Reading new GCode file:", gcode_file)
            with buffer_lock:
                gcode_buffer.append(GCode(gcode_file))
            processed_files.add(gcode_file)

        # Sleep for a while
        time.sleep(2)


### Code for rest endpoint to allow etch a sketch to check if gcode is available
app = Flask(__name__)


@app.route("/gcode/available")
def gcode_available():
    global gcode_buffer, buffer_lock
    with buffer_lock:
        # Return 200 if there is GCode in the buffer, else 204
        if len(gcode_buffer) > 0:
            return "", 200
        else:
            print("Returng 204")
            return "", 204


# Add click argument for gcode directory
@click.command()
@click.option(
    "--gcode_dir",
    default="gcode_outputs",
    help="Directory containing GCode files",
    type=click.Path(exists=True),
)
def main(gcode_dir: pathlib.Path):

    gcode_dir = pathlib.Path(gcode_dir)
    # Start the GCode reading thread
    gcode_thread = threading.Thread(target=get_new_gcode, args=(gcode_dir,))
    gcode_thread.start()

    # Start the Flask app to host the rest endpoint
    flask_thread = threading.Thread(
        target=app.run, kwargs={"host": "0.0.0.0", "port": 5001}
    )
    flask_thread.start()

    host, port = "0.0.0.0", 5000  # Your host and port here
    with socketserver.TCPServer((host, port), GCodeRequestHandler) as server:
        print("Server listening...")
        server.serve_forever()


if __name__ == "__main__":
    main()
