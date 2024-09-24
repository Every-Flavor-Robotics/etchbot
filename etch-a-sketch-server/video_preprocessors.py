import subprocess
from pathlib import Path
from click import secho
from PIL import Image
import numpy as np
import cv2
from filelock import FileLock, Timeout


# Base class for all preprocessors
from preprocessor_utils import Preprocessor

# Processor specific imports
from rembg import remove
from filelock import FileLock, Timeout


class VideoSplitters(Preprocessor):
    """Abstract class for video splitters.

    All video splitters separate a video into individual frames. They take in a video file and output a directory of images.
    """

    SUPPORTED_TYPES = [".mp4", ".mov", ".mkv", ".gif"]
    OUTPUT_EXTENSION = ""


class FFmpegSplitter(VideoSplitters):
    """Split a video into individual frames using FFmpeg.

    FFmpegSplitter is a class that takes in a video file and splits it into individual frames using FFmpeg. The output is a directory of images.
    """

    def __init__(self, output_framerate: int = 4):
        """Initialize the FFmpegSplitter.

        Args:
            output_framerate (int): Frame rate of the output images
        """

        # Confirm that output_framerate is an integer
        if not isinstance(output_framerate, int):
            raise ValueError("output_framerate must be an integer.")

        # Confirm that output_framerate is greater than 0
        if output_framerate <= 0:
            raise ValueError("output_framerate must be greater than 0.")

        self.output_framerate = output_framerate

        super().__init__()

    def _split_video(self, input_path: Path, output_dir: Path) -> Path:
        """Split the video into individual frames using FFmpeg.

        Args:
            input_path (Path): Path to the video file to split
            output_dir (Path): Path to the directory to save the frames

        Returns: None
        """

        # Define the output file name
        output_file = output_dir / "frame_%04d.png"

        # Define the FFmpeg command
        command = [
            "ffmpeg",
            "-i",
            str(input_path),
            "-vf",
            f"fps={self.output_framerate}",
            str(output_file),
        ]

        # Run the FFmpeg command
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        stdout, stderr = process.communicate()

        # Confirm that the FFmpeg command ran successfully
        if process.returncode != 0:
            raise ValueError(f"FFmpeg command failed: {stderr.decode('utf-8')}")

        return output_dir

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input video and return the output frames.

        Args:
            input_path (Path): Path to the input video file to process
            output_path (str): Name of the output directory

        Returns: Path to the output directory
        """

        super()._process(input_path, output_path)

        # Confirm that the input video exists
        if not input_path.exists():
            raise FileNotFoundError(f"Video {input_path} not found.")

        # Create the output directory
        output_path.mkdir(parents=True, exist_ok=True)

        # Get a lock for the output directory
        while True:
            try:
                with FileLock(output_path.with_suffix(".lock"), timeout=1.0):
                    self._split_video(input_path, output_path)
                    break
            except Timeout:
                continue

        # Confirm that the output directory exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output directory {output_path} not generated correctly."
            )

        return output_path
