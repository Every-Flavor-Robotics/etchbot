from flask import Flask, request, jsonify, send_file
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
    Drawing,
)
from etchbot import EtchBot, EtchBotStore
from http_camera import HTTPCamera
from config import Config


UPLOAD_DIR = Path("uploads")
PROCESSING_DIR = Path("processing")
OUTPUT_DIR = Path("gcode_outputs")

# Dictionary to store EtchBot instances
etchbot_store = EtchBotStore()

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
# Register the GCode blueprint, so that the GCode server can be started
# As a part of the main Flask app
app.register_blueprint(gcode_blueprint)


###### User facing API ######
@app.route("/upload/<etchbot_name>", methods=["POST"])
def upload_file(etchbot_name):
    print(f"Receiving file for EtchBot {etchbot_name}...")
    try:
        print(f"Receiving file for EtchBot {etchbot_name}...")
        file = request.files["file"]
        if not file:
            return jsonify({"error": "No file provided"}), 400

        # Check if the file type is supported
        # Do not save the file if it is not supported
        if not Drawing.is_supported_file_type(file.filename):
            return jsonify({"error": "Unsupported file type"}), 400

        # Confirm that that the Etchbot exists
        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)

        # Add the time
        save_path = (
            UPLOAD_DIR / etchbot_name / f"{time.strftime('%m%d%H%M%S')}_{file.filename}"
        )
        file.save(save_path)

        file_name = file.filename.split(".")[0]
        # Create a drawing and hand off to the Etchbot
        drawing = Drawing(file_name, save_path)
        etchbot.add_drawing(drawing)

        # Start file processing or any other logic related to the specific etchbot
        return (
            jsonify(
                {"message": f"File processing started for EtchBot {etchbot_name}."}
            ),
            200,
        )
    except Exception as e:
        print(f"Error during file upload for {etchbot_name}: {e}")
        return jsonify({"error": "Failed to process file"}), 500


# Get all connected EtchBots
@app.route("/etchbots", methods=["GET"])
def get_etchbots():
    names = etchbot_store.get_all_names()
    # Convert the set to a list and return it as a JSON response
    return jsonify({"etchbots": list(names)}), 200


# Get the state of a specific EtchBot
@app.route("/etchbot/state", methods=["GET"])
def get_etchbot_state():
    try:
        name = request.args.get("name")  # Get the name from query parameters
        if not name:
            return jsonify({"error": "No name provided"}), 400

        if name not in etchbot_store:
            return jsonify({"error": f"EtchBot {name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(name)

        return jsonify({"state": etchbot.state}), 200

    except Exception as e:
        print(f"Error during state retrieval: {e}")
        return jsonify({"error": "Failed to retrieve state"}), 500


@app.route("/etchbot/<etchbot_name>/status", methods=["GET"])
def get_etchbot_status(etchbot_name):
    # Return a JSON object containing the status of the EtchBot
    # Current state
    # Current drawing
    # Is camera connected
    # Is camera recording
    # Paused
    # Cooldown remaining

    try:
        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)

        drawing = etchbot.next_drawing()
        if drawing:
            drawing = drawing.name

        else:
            drawing = None

        status = {
            "state": etchbot.state,
            "current_drawing": drawing,
            "camera_connected": etchbot.camera is not None,
            "camera_recording": (
                etchbot.is_recording()
            ),
            "recording_mode": etchbot.record_while_drawing,
            "paused": etchbot.paused,
            "cooldown_remaining": etchbot.cooldown_remaining(),
        }

        return jsonify(status), 200

    except Exception as e:
        print(f"Error during status retrieval: {e}")
        return jsonify({"error": "Failed to retrieve status"}), 500


@app.route("/etchbot/queue", methods=["GET"])
def get_etchbot_queue():
    try:
        name = request.args.get("name")  # Get the name from query parameters
        if not name:
            return jsonify({"error": "No name provided"}), 400

        if name not in etchbot_store:
            return jsonify({"error": f"EtchBot {name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(name)

        return jsonify({"queue": etchbot.get_queue_information()}), 200

    except Exception as e:
        print(f"Error during queue retrieval: {e}")
        return jsonify({"error": "Failed to retrieve queue"}), 500


@app.route("/etchbot/completed", methods=["GET"])
def get_etchbot_completed():
    try:
        name = request.args.get("name")  # Get the name from query parameters
        if not name:
            return jsonify({"error": "No name provided"}), 400

        if name not in etchbot_store:
            return jsonify({"error": f"EtchBot {name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(name)

        return jsonify({"completed": etchbot.get_completed_information()}), 200

    except Exception as e:
        print(f"Error during completed drawings retrieval: {e}")
        return jsonify({"error": "Failed to retrieve completed drawings"}), 500


@app.route("/etchbot/<etchbot_name>/download_zip", methods=["POST"])
def download_artifact(etchbot_name):
    try:
        print(f"Downloading artifact for EtchBot {etchbot_name}...")
        data = request.get_json()
        print(data)

        drawing_index = data.get("drawing_index")
        if drawing_index is None:
            return jsonify({"error": "No index provided"}), 400

        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)

        artifact_path = etchbot.get_completed_zip(drawing_index)
        # Get name of the artifact
        artifact_name = etchbot.get_completed_information()[drawing_index]["name"]
        if not artifact_path or not artifact_path.exists():
            return (
                jsonify({"error": f"Artifact at index {drawing_index} not found"}),
                404,
            )

        # Send the file to the client for download
        return send_file(
            artifact_path, as_attachment=True, download_name=f"{artifact_name}.zip"
        )

    except Exception as e:
        print(f"Error downloading artifact: {e}")
        return jsonify({"error": "Failed to download artifact"}), 500


@app.route("/etchbot/<etchbot_name>/connect_camera", methods=["POST"])
def connect_camera(etchbot_name):
    try:
        data = request.get_json()
        service_url = data.get("service_url")
        camera_index = data.get("camera_index")

        if not service_url:
            return jsonify({"error": "No service URL provided"}), 400

        if camera_index is None:
            return jsonify({"error": "No camera index provided"}), 400

        try:
            camera = HTTPCamera(service_url, camera_index)
        except Exception as e:
            return jsonify({"error": f"Failed to connect to camera service: {e}"}), 500

        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)
        etchbot.set_camera(camera)

        return (
            jsonify({"message": f"Connected to camera service at {service_url}"}),
            200,
        )
    except Exception as e:
        print(f"Error connecting to camera service: {e}")
        return jsonify({"error": f"Failed to connect to camera service: {e}"}), 500


@app.route("/etchbot/<etchbot_name>/set_recording_mode", methods=["POST"])
def set_recording_mode(etchbot_name):
    try:
        data = request.get_json()
        recording_mode = data.get("recording_mode")

        if recording_mode is None:
            return jsonify({"error": "No recording mode provided"}), 400

        # Recording mode should be either true or false
        if recording_mode not in [True, False]:
            return jsonify({"error": "Invalid recording mode"}), 400

        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)

        if recording_mode:
            etchbot.enable_recording()

        else:
            etchbot.disable_recording()

        return (
            jsonify({"message": f"Recording mode set to {recording_mode}"}),
            200,
        )
    except Exception as e:
        print(f"Error setting recording mode: {e}")
        return jsonify({"error": f"Failed to set recording mode: {e}"}), 500


@app.route("/etchbot/<etchbot_name>/pause", methods=["POST"])
def pause_etchbot(etchbot_name):
    try:
        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)
        etchbot.pause()

        return jsonify({"message": "EtchBot paused"}), 200
    except Exception as e:
        print(f"Error pausing EtchBot: {e}")
        return jsonify({"error": f"Failed to pause EtchBot: {e}"}), 500


@app.route("/etchbot/<etchbot_name>/resume", methods=["POST"])
def resume_etchbot(etchbot_name):
    try:
        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)
        etchbot.resume()

        return jsonify({"message": "EtchBot resumed"}), 200
    except Exception as e:
        print(f"Error resuming EtchBot: {e}")
        return jsonify({"error": f"Failed to resume EtchBot: {e}"}), 500


@app.route("/etchbot/<etchbot_name>/clear_error", methods=["POST"])
def clear_error(etchbot_name):
    try:
        if etchbot_name not in etchbot_store:
            return jsonify({"error": f"EtchBot {etchbot_name} not found"}), 404

        etchbot = etchbot_store.get_robot_by_name(etchbot_name)
        etchbot.recover()

        return jsonify({"message": "EtchBot error cleared"}), 200
    except Exception as e:
        print(f"Error clearing EtchBot error: {e}")
        return jsonify({"error": f"Failed to clear EtchBot error: {e}"}), 500


### EtchBot API ###
# This API is used to communicate with the EtchBots


@app.route("/connect", methods=["POST"])
def connect():
    """API endpoint to connect an EtchBot.

    Expects a POST request with a JSON object containing the following fields:
    - name: The name of the EtchBot
    - ip: The IP address of the EtchBot

    Returns:
    - 200: If the EtchBot was successfully connected
    - 400: If the request is missing required fields
    - 500: If an error occurred while connecting the EtchBot
    """
    try:
        # Get the data from the request
        data = request.get_json()

        # Check if the request contains the required fields
        if not data:
            return jsonify({"error": "No data provided"}), 400

        if "name" not in data:
            return jsonify({"error": "No name provided"}), 400

        if "ip" not in data:

            return jsonify({"error": "No IP address provided"}), 400

        name = data["name"]
        ip = data["ip"].strip()

        # Check if the EtchBot instance already exists, if not create a new one
        etchbot = None
        if name not in etchbot_store:
            etchbot = EtchBot(ip, name)
            etchbot_store.add_robot(etchbot)

        else:
            # Confirm that the IP address matches the existing EtchBot
            etchbot = etchbot_store.get_robot_by_name(name)
            etchbot.update_ip(ip)

        # Connect the EtchBot
        try:
            if not etchbot.handle_connect_request():
                return jsonify({"error": "Failed to connect to EtchBot"}), 500

        except MachineError as e:
            return jsonify({"error": str(e)}), 500

        # Create a directory in UPLOAD_DIR for the EtchBot
        etchbot_dir = UPLOAD_DIR / name
        etchbot_dir.mkdir(exist_ok=True)

    except Exception as e:
        print(f"Error during connection: {e}")
        return jsonify({"error": f"Unknown error {e}"}), 500

    return jsonify({"message": "EtchBot connected"}), 200


@app.route("/command", methods=["GET"])
def get_command():
    """Get the command that an etchbot should execute.

    Expects a GET request a JSON object with the following fields:
    - name: The ip of the EtchBot

    Returns:
    - 200: If the command was successfully retrieved
    - 204: If there are no commands to execute
    - 500: If an error occurred while retrieving the command
    """

    try:
        # Check if the request contains the required fields
        # Get the name of the EtchBot, provided in the JSON body
        name = request.args.get("name")
        if not name:
            return jsonify({"error": "No name provided"}), 400

        # Check if the EtchBot instance already exists
        if name not in etchbot_store:
            return (
                jsonify(
                    {
                        "error": f"EtchBot {name} not found. Please call POST connect first"
                    }
                ),
                404,
            )

        # Get the EtchBot instance from the dictionary
        etchbot = etchbot_store.get_robot_by_name(name)

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
        if name not in etchbot_store:
            return (
                jsonify(
                    {"error": f"EtchBot {name} not found. Please call GET task first"}
                ),
                404,
            )

        # Get the EtchBot instance from the dictionary
        etchbot = etchbot_store.get_robot_by_name(name)

        # Notify the EtchBot that drawing is complete
        etchbot.drawing_complete(drawing_time=drawing_time)

        print(f"EtchBot {name} drawing completion notified.")

        return jsonify({"message": "Drawing completion notified"}), 200

    except Exception as e:
        print(f"Error during drawing completion: {e}")
        return jsonify({"error": f"Unknown error {e}"}), 500


@app.route("/erasing_complete", methods=["POST"])
def erasing_complete():
    """API endpoint to notify the server that an EtchBot has completed erasing.

    The request should contain a JSON object with the following fields:
    - name: The name of the EtchBot
    """

    try:
        # Get the data from the request
        data = request.get_json()

        # Check if the request contains the required fields
        if not data:
            return jsonify({"error": "No data provided"}), 400

        if "name" not in data:
            return jsonify({"error": "No name provided"}), 400

        name = data["name"]

        # Check if the EtchBot instance already exists, return an error if it doesn't
        if name not in etchbot_store:
            return (
                jsonify(
                    {"error": f"EtchBot {name} not found. Please call GET task first"}
                ),
                404,
            )

        # Get the EtchBot instance from the dictionary
        etchbot = etchbot_store.get_robot_by_name(name)

        # Notify the EtchBot that erasing is complete
        etchbot.erasing_complete()

        print(f"EtchBot {name} erasing completion notified.")

        return jsonify({"message": "Erasing completion notified"}), 200

    except Exception as e:
        print(f"Error during erasing completion: {e}")
        return jsonify({"error": f"Unknown error {e}"}), 500


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


@click.command()
@click.option(
    "--config",
    default="../etchbot_config.yml",
    help="Path to the YAML configuration file.",
)
def main(config):

    config = Config(config)

    # Confirm that the necessary directories exist
    if not UPLOAD_DIR.exists():
        UPLOAD_DIR.mkdir()

    if not PROCESSING_DIR.exists():
        PROCESSING_DIR.mkdir()

    if not OUTPUT_DIR.exists():
        OUTPUT_DIR.mkdir()

    run_gcode_server(OUTPUT_DIR, run_flask=False)

    print(f"Using configuration file: {config}")

    print("Registered Endpoints:")
    for rule in app.url_map.iter_rules():
        print(f"{rule.endpoint}: {rule.rule}")

    app.run("0.0.0.0", port=5010, use_reloader=False, debug=False)

if __name__ == "__main__":
    main()
