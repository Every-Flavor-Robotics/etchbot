from flask import Flask, request, jsonify
from flask_cors import CORS
from pathlib import Path
import io
import threading
import time
import shutil
import subprocess
from etch_a_sketch_cli import run_pipeline
from gcode_server import (
    gcode_blueprint,
    run_gcode_server,
)

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

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
app.register_blueprint(gcode_blueprint)


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


# Create a function that constantly checks UPLOAD DIR for new files
def process_files():
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


def gcode_server():
    print("GCode server started.")
    command = "python gcode_server.py --gcode_dir gcode_outputs"

    subprocess.run(command, shell=True)

    # Wait for the subprocess to finish
    print("GCode server finished.")
    time.sleep(1)


if __name__ == "__main__":
    # Confirm that the output directory exists
    if not UPLOAD_DIR.exists():
        UPLOAD_DIR.mkdir()

    if not PROCESSING_DIR.exists():
        PROCESSING_DIR.mkdir()

    if not OUTPUT_DIR.exists():
        OUTPUT_DIR.mkdir()

    # Start the processing thread
    processing_thread = threading.Thread(target=process_files)
    processing_thread.start()

    # Start the GCode server thread
    # gcode_thread = threading.Thread(target=gcode_server)
    # gcode_thread.start()

    run_gcode_server(OUTPUT_DIR, run_flask=False)

    app.run("0.0.0.0", port=5010, use_reloader=False, debug=False)


from flask import Flask, request, jsonify
from flask_cors import CORS
from pathlib import Path
import threading
import time
import shutil
import subprocess
from gcode_server import (
    gcode_blueprint,
    run_gcode_server,
)

UPLOAD_DIR = Path("uploads")
PROCESSING_DIR = Path("processing")
OUTPUT_DIR = Path("gcode_outputs")
SUPPORTED_IMAGE_TYPES = [".jpg", ".jpeg", ".png", ".svg"]
SUPPORTED_VIDEO_TYPES = [".mp4", ".avi", ".mov"]
SUPPORTED_GCODE_TYPES = [".gcode"]
SUPPORTED_FILE_TYPES = (
    SUPPORTED_IMAGE_TYPES + SUPPORTED_VIDEO_TYPES + SUPPORTED_GCODE_TYPES
)

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
app.register_blueprint(gcode_blueprint)


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


# Create a function that constantly checks UPLOAD DIR for new files
def process_files():
    print("Processing thread started.")
    while True:
        for file_path in UPLOAD_DIR.iterdir():
            try:
                print("Processing file:", file_path)
                file_extension = file_path.suffix.lower()
                if file_extension in SUPPORTED_IMAGE_TYPES:
                    processing_dir = PROCESSING_DIR / file_path.stem
                    run_pipeline(file_path, processing_dir, copy=False)

                    # Copy gcode to output directory, named input file name.gcode
                    gcode_path = processing_dir / "output.gcode"
                    output_path = OUTPUT_DIR / f"{file_path.stem}.gcode"
                    shutil.copy(gcode_path, output_path)
                    file_path.unlink()  # Remove the processed image
                elif (
                    file_extension in SUPPORTED_VIDEO_TYPES
                    or file_extension in SUPPORTED_GCODE_TYPES
                ):
                    # For video and GCode files, just move them to the processing directory
                    processing_dir = PROCESSING_DIR / file_path.name
                    shutil.move(str(file_path), processing_dir)
            except Exception as e:
                print(f"Error during file processing: {e}")

        time.sleep(1)


def gcode_server():
    print("GCode server started.")
    command = "python gcode_server.py --gcode_dir gcode_outputs"

    subprocess.run(command, shell=True)

    # Wait for the subprocess to finish
    print("GCode server finished.")
    time.sleep(1)


if __name__ == "__main__":
    # Confirm that the output directory exists
    if not UPLOAD_DIR.exists():
        UPLOAD_DIR.mkdir()

    if not PROCESSING_DIR.exists():
        PROCESSING_DIR.mkdir()

    if not OUTPUT_DIR.exists():
        OUTPUT_DIR.mkdir()

    # Start the processing thread
    processing_thread = threading.Thread(target=process_files)
    processing_thread.start()

    # Start the GCode server thread
    # gcode_thread = threading.Thread(target=gcode_server)
    # gcode_thread.start()

    run_gcode_server(OUTPUT_DIR, run_flask=False)

    app.run("0.0.0.0", port=5010, use_reloader=False, debug=False)
