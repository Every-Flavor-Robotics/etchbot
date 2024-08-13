from flask import Flask, request, jsonify
from flask_cors import CORS
from pathlib import Path
import datetime
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


@app.route("/update_state", methods=["POST"])
def update_state():
    """API endpoint to update the state of an EtchBot.

    The request should contain a JSON object with the following fields:
    - name: The name of the EtchBot
    - current_state: The state to transition to
    - timestamp: The timestamp of the state update

    Example request:
    {
        "name": "EtchBot1",
        "current_state": "Idle",
        "timestamp": "2021-08-15T12:00:00Z"
    }

    """

    try:
        data = request.get_json()
        # Check if the request contains the required fields
        if (
            not data
            or "name" not in data
            or "current_state" not in data
            or "timestamp" not in data
        ):
            return jsonify({"error": "Invalid data"}), 400

        name = data["name"]
        current_state = data["current_state"]
        timestamp = data.get("timestamp", datetime.utcnow().isoformat() + "Z")

        # Check if the EtchBot instance already exists, if not create a new one
        if name not in etchbots:
            etchbots[name] = EtchBot(name)

        # Get the EtchBot instance from the dictionary
        etchbot = etchbots[name]

        # Check if the current state is valid and trigger the corresponding transition
        if hasattr(etchbot, current_state.lower()):
            getattr(etchbot, current_state.lower())()

        print(f"EtchBot {name} state updated to {current_state} at {timestamp}")
        return (
            jsonify(
                {"message": f"State updated to {current_state}", "timestamp": timestamp}
            ),
            200,
        )

    except Exception as e:
        print(f"Error updating state: {e}")
        return jsonify({"error": "Failed to update state"}), 500


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
