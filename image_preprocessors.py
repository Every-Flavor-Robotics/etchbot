import subprocess
from abc import ABC, abstractmethod
from pathlib import Path
from click import secho

from PIL import Image


class ImagePreprocessor(ABC):
    """Abstract class for image preprocessors.

    All image preprocessors take a Path to an image as an input and return a PIL Image as an output. The process method is
    abstract and must be implemented by the subclass and the output is the image that will be fed to the next step
    in the pipeline.
    """

    def __init__(self):
        pass

    @abstractmethod
    def process(self, image_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            image_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns:
            Path: Path to the processed image, ready for the next step in the pipeline
        """

        # Print class name and "Running..." in green
        secho(f"\tRunning {self.__class__.__name__}...", fg="green")

class AspectRatioPreprocessor(ImagePreprocessor):
    """Aspect Ratio Preprocessor crops the input image to a specified size."""

    def __init__(self, aspect_ratio: float, resize: bool = False):
        """Initialize the Crop Preprocessor with the crop size.

        Args:
            aspect_ratio (float): Aspect ratio to crop the image to (width / height)
            resize (bool): Whether to resize the image to the crop size
        """
        self.aspect_ratio = aspect_ratio
        self.resize = resize


    def process(self, image_path: Path, output_path: Path) -> None:
        """Process the input image and return the output image.

        Args:
            image_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns:
            Path: Path to the processed image, ready for the next step in the pipeline
        """
        super().process(image_path, output_path)

        # Confirm that the input image exists
        if not image_path.exists():
            raise FileNotFoundError(f"Image {image_path} not found.")

        # Load the image
        image = Image.open(image_path)

        if self.resize:
            # Resize the image to the correct aspect ratio
            image = self.resize(image)
        else:
            # Crop the center of the image to the correct aspect ratio
            image = self.crop_center(image)

        # Save the cropped image
        image.save(output_path)

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )


    def compute_new_dimensions(self, width: int, height: int) -> tuple[int, int]:
        """Compute the new dimensions of the image after cropping."""

        # Compute the new dimensions
        new_width = min(width, height * self.aspect_ratio)
        new_height = min(height, width / self.aspect_ratio)

        return new_width, new_height

    def crop_center(self, img: Image) -> Image:
        """ Crop the center of the image to the specified aspect ratio."""

        # Get the dimensions of the image
        width, height = img.size

        # Compute the new dimensions
        new_width, new_height = self.compute_new_dimensions(width, height)

        # Calculate the dimensions of the center crop
        left = (width - new_width) / 2
        top = (height - new_height) / 2
        right = (width + new_width) / 2
        bottom = (height + new_height) / 2

        # Crop the center of the image
        img = img.crop((left, top, right, bottom))

        return img

    def resize(self, img: Image) -> Image:
        """Resize the image to the correct aspect ratio."""

        # Get the dimensions of the image
        width, height = img.size

        # Compute the new dimensions
        new_width, new_height = self.compute_new_dimensions(width, height)


        # Resize the image
        img = img.resize((new_width, new_height), Image.ANTIALIAS)

        return img


class ColorbookPreprocessor(ImagePreprocessor):
    """Colorbook Preprocessor generates contours from the input image, like a coloring book."""

    COLORBOOK_PATH = "~/efr/GsColorbook/python-implementation/colorbook_cli.py"

    def process(self, image_path: Path, output_path: Path) -> None:
        """Process the input image and return the output image.

        Args:
            image (Image): Raw image to be processed

        Returns:
            Path: Path to the processed image, ready for the next step in the pipeline
        """
        super().process(image_path, output_path)


        # Confirm that the input image exists
        if not image_path.exists():
            raise FileNotFoundError(f"Image {image_path} not found.")

        # Temp directory to store the output of the colorbook preprocessor
        # Remove the image name and extension from the output path
        output_path = output_path.absolute()
        output_temp_dir = output_path.parent / "temp"

        # Construct the command to run the colorbook preprocessor
        command = f"python {self.COLORBOOK_PATH} -i {image_path.absolute()} -o {output_temp_dir}"

        # Run the command
        try:
            subprocess.run(command, check=True, shell=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Preprocessor execution failed: {e}") from e

        # Transfer outlined.png to the output directory
        outlined_path = output_temp_dir / "outlined.png"
        subprocess.run(f"mv {outlined_path} {output_path}", check=True, shell=True)

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )

        # Remove the temp directory
        subprocess.run(f"rm -r {output_temp_dir}", check=True, shell=True)