from flask import Flask, request
from PIL import Image
import io
from pathlib import Path
from etch_a_sketch_cli import run_pipeline
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

app = Flask(__name__)
app.register_blueprint(gcode_blueprint)


@app.route("/upload", methods=["POST"])
def upload_file():
    print("Receiving image...")
    file = request.files["image"]

    image = Image.open(io.BytesIO(file.read()))

    # Name image by timestamp MMDDHHMMSS

    save_path = UPLOAD_DIR / f"{time.strftime('%m%d%H%M%S')}.png"

    image.save(save_path)

    return "Image processing started."


# Create a function that constantly checks UPLOAD DIR for new files
def process_images():
    print("Processing thread started.")
    while True:
        for image_path in UPLOAD_DIR.iterdir():
            print("Processing image:", image_path)
            if image_path.suffix in [".jpg", ".jpeg", ".png"]:
                processing_dir = PROCESSING_DIR / image_path.stem
                run_pipeline(image_path, processing_dir, copy=False)

                # Copy gcode to output directory, named input file name.gcode
                gcode_path = processing_dir / "output.gcode"
                output_path = OUTPUT_DIR / f"{image_path.stem}.gcode"
                shutil.copy(gcode_path, output_path)

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
    processing_thread = threading.Thread(target=process_images)
    processing_thread.start()

    # Start the GCode server thread
    # gcode_thread = threading.Thread(target=gcode_server)
    # gcode_thread.start()

    run_gcode_server(OUTPUT_DIR, run_flask=False)

    app.run("0.0.0.0", port=5001, use_reloader=False, debug=False)

