from flask import Flask, request, jsonify
from flask_cors import CORS
from transitions import MachineError
from pathlib import Path
from datetime import datetime
import threading
import time
import shutil
from etch_a_sketch_cli import run_pipeline
from gcode_server import (
    gcode_blueprint,
    run_gcode_server,
)
from etchbot import EtchBot

UPLOAD_DIR = Path("uploads")
PROCESSING_DIR = Path("processing")
OUTPUT_DIR = Path("gcode_outputs")
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

# Dictionary to store EtchBot instances
etchbots = {}

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
# Register the GCode blueprint, so that the GCode server can be started
# As a part of the main Flask app
app.register_blueprint(gcode_blueprint)


###### User facing API ######
@app.route("/upload", methods=["POST"])
def upload_file():
    try:
        print("Receiving file...")
        file = request.files["file"]
        if not file:
            return jsonify({"error": "No file provided"}), 400

        file_extension = Path(file.filename).suffix.lower()

        if file_extension not in SUPPORTED_FILE_TYPES:
            return jsonify({"error": "Unsupported file type"}), 400

        save_path = UPLOAD_DIR / f"{time.strftime('%m%d%H%M%S')}{file_extension}"
        file.save(save_path)

        return jsonify({"message": "File processing started."}), 200
    except Exception as e:
        print(f"Error during file upload: {e}")
        return jsonify({"error": "Failed to process file"}), 500


def process_files():
    """Function that constantly checks UPLOAD DIR for new files and processes them.

    Supported file types:
    - Images: .jpg, .jpeg, .png, .svg
    - Videos: .mp4, .avi, .mov
    - GCode: .gcode
    - Optimized GCode: .optgcode <- bypasses all processing and is directly sent to the GCode server

    """
    print("Processing thread started.")
    while True:
        for file_path in UPLOAD_DIR.iterdir():
            try:
                print("Processing file:", file_path)
                file_extension = file_path.suffix.lower()
                if file_extension in SUPPORTED_OPTIMIZED_GCODE_TYPES:
                    # For optimized GCode files, just move them to the output directory
                    # Change the file extension to .gcode
                    output_path = OUTPUT_DIR / f"{file_path.stem}.gcode"
                    shutil.move(str(file_path), output_path)

                # If it's any other supported file type, process it
                elif file_extension in SUPPORTED_FILE_TYPES:
                    processing_dir = PROCESSING_DIR / file_path.stem
                    run_pipeline(file_path, processing_dir, copy=False)

                    # Copy gcode to output directory, named input file name.gcode
                    # TODO: Update to support directories too
                    gcode_path = processing_dir / "output.gcode"
                    output_path = OUTPUT_DIR / f"{file_path.stem}.gcode"
                    shutil.copy(gcode_path, output_path)
                    file_path.unlink()
                # Do nothing if file type is not supported
            except Exception as e:
                print(f"Error during file processing: {e}")

        time.sleep(1)


### EtchBot API ###
# This API is used to communicate with the EtchBots


@app.route("/command", methods=["GET"])
def get_command():
    """Get the command that an etchbot should execute.

    Expects a GET request a JSON object with the following fields:
    - name: The name of the EtchBot

    Returns:
    - 200: If the command was successfully retrieved
    - 204: If there are no commands to execute
    - 500: If an error occurred while retrieving the command
    """

    try:
        # Check if the request contains the required fields
        name = request.args.get("name")
        if not name:
            return jsonify({"error": "No name provided"}), 400

        # Check if the EtchBot instance already exists
        if name not in etchbots:
            etchbots[name] = EtchBot(name)

        # Get the EtchBot instance from the dictionary
        etchbot = etchbots[name]

        # Get the command to be executed by the EtchBot
        command = etchbot.get_command()

        # Send command if it exists
        if command:
            print(f"EtchBot {name} command retrieved: {command}")
            return jsonify({"command": command}), 200

        # Return 204 if no command to execute
        return jsonify({"message": "No command to execute"}), 204

    except Exception as e:
        print(f"Error during command retrieval: {e}")
        return jsonify({"error": "Failed to retrieve command"}), 500


@app.route("/drawing_complete", methods=["POST"])
def drawing_complete():
    """API endpoint to notify the server that an EtchBot has completed drawing.

    The request should contain a JSON object with the following fields:
    - name: The name of the EtchBot
    - drawing_time: The time taken to complete the drawing in seconds
    """
    try:
        # Get the data from the request
        data = request.get_json()

        # Check if the request contains the required fields
        if not data:
            return jsonify({"error": "No data provided"}), 400

        if "name" not in data:
            return jsonify({"error": "No name provided"}), 400

        if "drawing_time" not in data:
            return jsonify({"error": "No drawing time provided"}), 400

        name = data["name"]
        drawing_time = data["drawing_time"]

        # Check if the EtchBot instance already exists, return an error if it doesn't
        if name not in etchbots:
            return (
                jsonify(
                    {"error": f"EtchBot {name} not found. Please call GET task first"}
                ),
                404,
            )

        # Get the EtchBot instance from the dictionary
        etchbot = etchbots[name]

        # Notify the EtchBot that drawing is complete
        etchbot.drawing_complete(drawing_time)

        print(f"EtchBot {name} drawing completion notified.")

    except Exception as e:
        print(f"Error during drawing completion: {e}")
        return jsonify({"error": f"Unknown error {e}"}), 500


@app.route("/erasing_complete", methods=["POST"])
def erasing_complete():
    """API endpoint to notify the server that an EtchBot has completed erasing.

    The request should contain a JSON object with the following fields:
    - name: The name of the EtchBot
    - timestamp: The timestamp of the erasing completion
    """
    pass


@app.route("/error", methods=["POST"])
def error():
    pass


@app.route("/update_state", methods=["POST"])
def update_state():
    """API endpoint to update the state of an EtchBot.

    The request should contain a JSON object with the following fields:
    - name: The name of the EtchBot
    - transition: The state to transition to
    - timestamp: The timestamp of the state update

    Example request:
    {
        "name": "EtchBot1",
        "transition": "connect",
        "timestamp": "2021-08-15T12:00:00Z"
    }

    """
    try:
        data = request.get_json()

        # Check if the request contains the required fields
        if not data:
            return jsonify({"error": "No data provided"}), 400

        if "name" not in data:
            return jsonify({"error": "No name provided"}), 400

        if "transition" not in data:
            return jsonify({"error": "No transition provided"}), 400

        if "timestamp" not in data:
            return jsonify({"error": "No timestamp provided"}), 400

        name = data["name"]
        transition = data["transition"]
        timestamp = data["timestamp"]

        # Check if the EtchBot instance already exists, if not create a new one
        if name not in etchbots:
            etchbots[name] = EtchBot(name)

        # Get the EtchBot instance from the dictionary
        etchbot = etchbots[name]

        # Check if the current state is valid and trigger the corresponding transition
        if hasattr(etchbot, transition.lower()):
            try:
                # Trigger the transition
                getattr(etchbot, transition.lower())()
            except MachineError as e:
                return jsonify({"error": str(e)}), 400
        else:
            # Return an error if the state is invalid
            return (
                jsonify({"error": f"Received invalid state: {transition.lower()}"}),
                400,
            )

        current_state = etchbot.state
        print(
            f"EtchBot {name} transition {transition}. Executed. State updated to {current_state} at {timestamp}"
        )

        return (
            jsonify(
                {"message": f"State updated to {current_state}", "timestamp": timestamp}
            ),
            200,
        )

    except Exception as e:
        print(f"Error during state update: {e}")
        return jsonify({f"error": "Unknown error during update - {e}"}), 500


if __name__ == "__main__":
    # Confirm that the necessary directories exist
    if not UPLOAD_DIR.exists():
        UPLOAD_DIR.mkdir()

    if not PROCESSING_DIR.exists():
        PROCESSING_DIR.mkdir()

    if not OUTPUT_DIR.exists():
        OUTPUT_DIR.mkdir()

    # Start the processing thread
    processing_thread = threading.Thread(target=process_files)
    processing_thread.start()

    run_gcode_server(OUTPUT_DIR, run_flask=False)

    print("Registered Endpoints:")
    for rule in app.url_map.iter_rules():
        print(f"{rule.endpoint}: {rule.rule}")

    app.run("0.0.0.0", port=5010, use_reloader=False, debug=False)
