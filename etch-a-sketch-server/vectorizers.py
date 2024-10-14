from abc import ABC, abstractmethod
from pathlib import Path
from click import secho
import numpy as np
from PIL import Image

# Base class for all preprocessors
from preprocessor_utils import Preprocessor

from potrace import TURNPOLICY_MINORITY, Bitmap  # `potracer` library
import vtracer


class Vectorizer(Preprocessor):
    """Abstract class for vectorizers.

    All vectorizers take a Path to an image as an input and return a Path to a vectorized image as an output. The process
    method is abstract and must be implemented by the subclass and the output is the image that will be fed to the next
    step in the pipeline.
    """

    SUPPORTED_TYPES = [".png", ".jpg", ".jpeg"]
    OUTPUT_EXTENSION = ".svg"


class PotraceVectorizer(Vectorizer):
    """PotraceVectorizer uses potrace to vectorize the input image."""

    PARALLELIZABLE = True

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            image_path (Path): Path to the image to be processed
            output_path (Path): Path to save the processed image

        Returns:
            Path: Path to the processed image, ready for the next step in the pipeline
        """

        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        # Open image with PIL
        image = Image.open(input_path)

        image_array = np.array(image) / 255.0

        # Quantize everything > 0.5 to 1
        image_array[image_array > 0.8] = 1
        # Quantize everything <= 0.5 to 0
        image_array[image_array < 1] = 0

        # Pass np array to potrace
        # Convert to black and white
        bm = Bitmap(np.array(image_array))

        # Vectorize the image
        path = bm.trace(
            turdsize=20, alphamax=1, opttolerance=0.2, turnpolicy=TURNPOLICY_MINORITY
        )

        # Create the output file
        with open(output_path, "w") as f:
            f.write(
                f"""<svg version="1.1" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{image.width}" height="{image.height}" viewBox="0 0 {image.width} {image.height}">"""
            )
            parts = []
            for curve in path:
                fs = curve.start_point
                fs_x, fs_y = fs
                parts.append(f"M{fs_x},{fs_y}")
                for segment in curve.segments:
                    if segment.is_corner:
                        a_x, a_y = segment.c
                        b_x, b_y = segment.end_point
                        parts.append(f"L{a_x},{a_y}L{b_x},{b_y}")
                    else:
                        a_x, a_y = segment.c1
                        b_x, b_y = segment.c2
                        c_x, c_y = segment.end_point
                        parts.append(f"C{a_x},{a_y} {b_x},{b_y} {c_x},{c_y}")
                parts.append("z")
            f.write(
                f'<path stroke="#000000" fill="None" fill-rule="evenodd" d="{"".join(parts)}"/>'
            )
            f.write("</svg>")

        return output_path


class VTracerVectorizer(Vectorizer):
    """VTracerVectorizer uses vtracer to vectorize the input image."""

    PARALLELIZABLE = True

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.py

        Args:
            image_path (Path): Path to the image to be processed
            output_path (Path): Path to save the processed image

        Returns:
            Path: Path to the processed image, ready for the next step in the pipeline

        """

        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        # Vectorize the image
        vtracer.convert_image_to_svg_py(input_path, output_path, colormode="binary")

        return output_path
