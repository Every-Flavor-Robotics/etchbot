import pathlib
import socketserver
import time
import click
import threading
import socket
from flask import Flask, Blueprint, jsonify
import re
from filelock import FileLock, Timeout
from click import secho


gcode_buffer = []
buffer_lock = threading.Lock()

status_UNKNOWN = "UNKNOWN"
status_READY = "READY"
status_DRAWING = "DRAWING"
status_ERROR = "ERROR"
cur_status = status_UNKNOWN


def write_robot_status(status):
    global cur_status
    if status == cur_status:
        return

    cur_status = status
    while True:
        try:
            with FileLock("robot_status.txt.lock"):
                with open("robot_status.txt", "w") as f:
                    f.write(status)
        except Timeout:
            print("Failed to acquire lock on robot_status.txt")
        break


class GCode:
    def __init__(self, gcode_file, lines=None, parent_dir=None):
        self.gcode_file = gcode_file
        self.parent_dir = parent_dir

        if lines is not None:
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
        if self.parent_dir and not any(self.parent_dir.iterdir()):
            self.parent_dir.rmdir()

    def reset(self):
        self.line_number = 0


class EmptyGCode(GCode):
    def __init__(self):
        super().__init__(None, ["END\n"])


class GCodeRequestHandler(socketserver.StreamRequestHandler):
    start_line = 0

    def handle(self):
        global gcode_buffer, buffer_lock, status_READY, status_DRAWING, status_ERROR

        print("Connected by", self.client_address)

        write_robot_status(status_DRAWING)

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
                    gcode_block = gcode.get_gcode(num_lines_requested)

                    print("Sending response")
                    self.request.sendall(gcode_block.encode())

                    print("Response sent")
            gcode.delete_file()
        except socket.timeout:
            print("Connection timed out. Will retry this gcode next time")
            gcode.reset()
            with buffer_lock:
                gcode_buffer.insert(0, gcode)
            write_robot_status(status_ERROR)
        except (ConnectionResetError, BrokenPipeError):
            print("Connection closed by client. Will retry this gcode next time")
            gcode.reset()
            with buffer_lock:
                gcode_buffer.insert(0, gcode)
        print("G-code transmission ended. Closing connection.")
        self.request.close()


def get_new_gcode(scan_directory: pathlib.Path):
    global gcode_buffer, buffer_lock

    print("Scanning directory:", scan_directory)

    processed_files = set()
    while True:
        new_gcode_files = list(scan_directory.glob("*.gcode"))
        new_gcode_files = [
            gcode_file
            for gcode_file in new_gcode_files
            if gcode_file not in processed_files
        ]

        for gcode_file in new_gcode_files:
            print("Reading new GCode file:", gcode_file)
            with buffer_lock:
                gcode_buffer.append(GCode(gcode_file))
            processed_files.add(gcode_file)

        time.sleep(2)


def extract_number(filename: str):
    match = re.search(r"\d+", filename)
    return int(match.group()) if match else float("inf")


gcode_blueprint = Blueprint("gcode_blueprint", __name__)


@gcode_blueprint.route("/gcode/available")
def gcode_available():
    global gcode_buffer, buffer_lock
    with buffer_lock:
        if len(gcode_buffer) > 0:
            return "", 200
        else:
            print("Returning 204")
            write_robot_status(status_READY)
            return "", 204


def start_gcode_server(host="0.0.0.0", port=5000):
    # Print in green, starting the GCode server
    secho("GCode Streamer: Starting up", fg="green")
    print("\tGCode Streamer: Host:", host)
    print("\tGCode Streamer: Port:", port)
    with socketserver.TCPServer((host, port), GCodeRequestHandler) as server:
        secho("GCode Streamer: Server listening...", fg="green")
        server.serve_forever()


def run_gcode_server(scan_directory, run_flask=True):
    threading.Thread(target=get_new_gcode, args=(scan_directory,)).start()
    threading.Thread(target=start_gcode_server).start()

    if run_flask:
        app = Flask(__name__)
        app.register_blueprint(gcode_blueprint)
        app.run("0.0.0.0", port=5001, use_reloader=False, debug=False)


@click.command()
@click.option(
    "--gcode_dir",
    default="gcode_outputs",
    help="Directory containing GCode files",
    type=click.Path(exists=True),
)
def main(gcode_dir: pathlib.Path):
    gcode_dir = pathlib.Path(gcode_dir)
    if not gcode_dir.exists():
        raise FileNotFoundError(f"GCode directory {gcode_dir} not found.")
    run_gcode_server(gcode_dir)


if __name__ == "__main__":
    main()
