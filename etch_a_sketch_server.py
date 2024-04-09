from flask import Flask, request
from PIL import Image
import io
from pathlib import Path
from etch_a_sketch_cli import run_pipeline
import threading
import time
import shutil


UPLOAD_DIR = Path("uploads")
PROCESSING_DIR = Path("processing")
OUTPUT_DIR = Path("gcode_outputs")

app = Flask(__name__)

@app.route('/upload', methods=['POST'])
def upload_file():
    file = request.files['image']

    image = Image.open(io.BytesIO(file.read()))

    # Name image by timestamp MMDDHHMMSS

    save_path = UPLOAD_DIR / f"{time.strftime('%m%d%H%M%S')}.png"


    return 'Image processing started.'

# Create a function that constantly checks UPLOAD DIR for new files
def process_images():
    while True:
        for image_path in UPLOAD_DIR.iterdir():
            if image_path.suffix in ['.jpg', '.jpeg', '.png']:
                processing_dir = PROCESSING_DIR / image_path.stem
                processing_dir.mkdir()
                run_pipeline(image_path, processing_dir)

                # Delete the image after processing
                image_path.unlink()

                # Copy gcode to output directory, named input file name.gcode
                gcode_path = processing_dir / "output.gcode"
                output_path = OUTPUT_DIR / f"{image_path.stem}.gcode"
                shutil.copy(gcode_path, output_path)


        time.sleep(1)


if __name__ == '__main__':
    # Confirm that the output directory exists
    if not UPLOAD_DIR.exists():
        UPLOAD_DIR.mkdir()

    if not PROCESSING_DIR.exists():
        PROCESSING_DIR.mkdir()

    if not OUTPUT_DIR.exists():
        OUTPUT_DIR.mkdir()

    # Start the processing thread
    processing_thread = threading.Thread(target=process_images)


    app.run(debug=True)