import pathlib
import socketserver
import time
import click
import threading
import socket
from flask import Flask, Blueprint, jsonify, request
import re
from filelock import FileLock, Timeout
from click import secho
from etch_a_sketch_cli import run_pipeline
from pathlib import Path
import shutil
from etchbot import EtchBotStore
from config import Config


etchbot_store = EtchBotStore()
config = Config()

SUPPORTED_IMAGE_TYPES = [".jpg", ".jpeg", ".png", ".svg"]
SUPPORTED_VIDEO_TYPES = [".mp4", ".avi", ".mov"]
SUPPORTED_GCODE_TYPES = [".gcode"]
# optgcode bypasses all processing and is directly sent to the GCode server
SUPPORTED_OPTIMIZED_GCODE_TYPES = [".optgcode"]
SUPPORTED_FILE_TYPES = (
    SUPPORTED_IMAGE_TYPES
    + SUPPORTED_VIDEO_TYPES
    + SUPPORTED_GCODE_TYPES
    + SUPPORTED_OPTIMIZED_GCODE_TYPES
)

UPLOAD_DIR = Path("uploads")
PROCESSING_DIR = Path("processing")


def extract_number(filename: str):
    match = re.search(r"\d+", filename)
    return int(match.group()) if match else float("inf")


def extract_gcode_coords(line):
    coords = [None, None, None]
    if line.startswith('G1') or line.startswith('G0') or line.startswith('G00') or line.startswith('G01'):
        x_match = re.search(r'X(-?\d+(\.\d+)?)', line)
        y_match = re.search(r'Y(-?\d+(\.\d+)?)', line)
        z_match = re.search(r'Z(-?\d+(\.\d+)?)', line)
        coords[0] = float(x_match.group(1)) if x_match else None
        coords[1] = float(y_match.group(1)) if y_match else None
        coords[2] = float(z_match.group(1)) if z_match else None

    return coords



class GCode:
    def __init__(self, gcode_file, lines=None, parent_dir=None):
        self.gcode_file = gcode_file
        self.parent_dir = parent_dir

        if lines is not None:
            self.gcode_lines = lines
            self.loaded = True
        else:
            # Confirm that the file exists
            if not self.gcode_file.exists():
                raise FileNotFoundError(f"File {self.gcode_file} not found.")

            self.loaded = False

            self.gcode_lines = []

        self.line_number = 0

    def load(self):
        with open(self.gcode_file) as f:
            self.gcode_lines = f.readlines()

            # Get coords of the last line
            last_line = self.gcode_lines[-1]
            last_coords = extract_gcode_coords(last_line)
            # Remove it if it's returning to home (0,0)
            if last_coords[0] == 0 and last_coords[1] == 0:
                self.gcode_lines.pop()

            zero_point = config.get("drawing.zero", 0.0)

            self.gcode_lines.append(f"G0 X{zero_point} Y{zero_point}\n")
            self.gcode_lines.append("G28\n")
            self.gcode_lines.append("END\n")

        print(f"Read in {len(self.gcode_lines)} lines from {self.gcode_file}.")
        self.loaded = True

    def get_gcode(self, num_lines):
        if self.loaded == False:
            self.load()

        gcode_block = "".join(
            self.gcode_lines[self.line_number : self.line_number + num_lines]
        )
        self.line_number += num_lines
        return gcode_block

    def complete(self):
        if not self.loaded:
            return False

        return self.line_number >= len(self.gcode_lines)

    def delete_file(self):
        if self.gcode_file is not None:
            self.gcode_file.unlink()
        if self.parent_dir and not any(self.parent_dir.iterdir()):
            self.parent_dir.rmdir()

    def reset(self):
        self.line_number = 0

    def get_name(self):
        return self.gcode_file.stem


class EmptyGCode(GCode):
    def __init__(self):
        super().__init__(None, ["END\n"])


class Drawing:
    ARTIFACTS_DIR = Path("artifacts")

    def __init__(self, name: str, path: pathlib.Path):
        self.name = name

        self.path = path

        # Confirm that the file exists
        if not self.path.exists():
            raise FileNotFoundError(f"File {self.path} not found.")

        # Check if the file is a supported type
        file_extension = self.path.suffix.lower()
        if file_extension not in SUPPORTED_FILE_TYPES:
            raise ValueError(f"File type {file_extension} not supported.")

        self.ready = False

        # List of GCodes related to this drawing
        self.gcodes = []

        self.artifact_dir = None

    def start_processing(self):
        # Start processing the files in the directory in a separate thread
        threading.Thread(target=self._process_helper).start()

    def _process_helper(self):
        # Process the files in the directory
        try:
            print("Processing file:", self.path)

            output = None
            file_extension = self.path.suffix.lower()
            if file_extension in SUPPORTED_FILE_TYPES:
                processing_dir = PROCESSING_DIR / self.path.stem
                output = run_pipeline(self.path, processing_dir, copy=False)

        # Do nothing if file type is not supported
        except Exception as e:
            print(f"Error during file processing: {e}")

        self._add_gcode_to_queue(output)

        # Create an artifact directory for the drawing
        self.artifact_dir = processing_dir / self.ARTIFACTS_DIR
        self.artifact_dir.mkdir(exist_ok=True)

        self.processing_dir = processing_dir

        self.ready = True

    def get_artifiact_dir(self):
        return self.artifact_dir

    def _add_gcode_to_queue(self, output: pathlib.Path):
        if output is None:
            return

        # Check if output is a directory
        if output.is_dir():
            # Add the GCode files in the directory to the queue in order
            gcode_files = sorted(
                output.glob("*.optgcode"), key=lambda x: extract_number(x.stem)
            )

            self.gcodes.extend(
                [GCode(gcode_file, parent_dir=output) for gcode_file in gcode_files]
            )

        # Check if output is a file
        elif output.is_file():
            # Add the GCode file to the queue
            self.gcodes.append(GCode(output))

        # Do nothing if output is neither a file nor a directory
        print("GCodes added to queue:", len(self.gcodes))

    def status(self):
        # Return whether or not drawing is ready, and the number of GCodes in the queue
        return self.ready, len(self.gcodes)

    def next(self):
        if len(self.gcodes) == 0:
            return None

        return self.gcodes[0]

    def pop(self):
        if len(self.gcodes) == 0:
            return None

        return self.gcodes.pop(0)

    def get_zipped_artifact(self) -> pathlib.Path:
        # Name of archive should be name of processing dir
        name = self.processing_dir.stem
        archive_path = shutil.make_archive(name, "zip", self.processing_dir)

        return Path(archive_path)

    def __len__(self):
        return len(self.gcodes)

    def name(self):
        return self.name

    @staticmethod
    def is_supported_file_type(file_path: [pathlib.Path, str]):
        if isinstance(file_path, str):
            file_path = pathlib.Path(file_path)
        return file_path.suffix.lower() in SUPPORTED_FILE_TYPES


class GCodeRequestHandler(socketserver.StreamRequestHandler):
    start_line = 0

    def handle(self):
        global etchbot_store

        print("Connected by", self.client_address)

        etchbot = etchbot_store.get_robot_by_ip(self.client_address[0])
        if etchbot is None:
            print("Etchbot not found. Closing connection.")
            self.request.close()
            return

        gcode = etchbot.next_gcode()
        if gcode is None:
            print("No GCode found. Sending empty GCode.")
            # Return an empty GCode file
            gcode = EmptyGCode()

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

        except socket.timeout:
            print("Connection timed out. Will retry this gcode next timee")
            gcode.reset()
        except (ConnectionResetError, BrokenPipeError):
            print("Connection closed by client. Will retry this gcode next time")
            gcode.reset()
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


gcode_blueprint = Blueprint("gcode_blueprint", __name__)


@gcode_blueprint.route("/gcode/available")
def gcode_available():
    global etchbot_store
    try:
        # Check if the etchbot name is in the request
        etchbot_name = request.args.get("name")
        if etchbot_name is None:
            return "Error: Etchbot name not provided", 400

        etchbot = EtchBotStore().get_robot_by_name(etchbot_name)
        if etchbot is None:
            return f"Error: Etchbot {etchbot_name} not found", 404

        if etchbot.gcode_ready():
            return jsonify({"available": True}), 200

        else:
            print("Returning 204")
            return jsonify({"available": False}), 204
    except Exception as e:
        print(e)
        return f"Error: {e}", 500


def start_gcode_server(host="0.0.0.0", port=5005):
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
        app.run("0.0.0.0", port=5010, use_reloader=False, debug=False)


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
