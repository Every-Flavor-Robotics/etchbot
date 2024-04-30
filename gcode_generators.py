import subprocess
from abc import ABC, abstractmethod
from pathlib import Path
import click


class GCodeGenerator(ABC):
    """Abstract class for GCode Generators.

    All GCode Generators take a Path to an image as an input and return a Path to a GCode file as an output. The process
    method is abstract and must be implemented by the subclass and the output is the GCode file that will be fed to the
    next step in the pipeline.
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
        click.secho(f"\tRunning {self.__class__.__name__}...", fg="green")


class Svg2GcodeGenerator(GCodeGenerator):
    """Svg2gcodeGenerator uses svg2gcode to convert the input SVG to GCode.

    https://github.com/sameer/svg2gcode.git
    """

    SVG2GCODE_PATH = "~/efr/svg2gcode/Cargo.toml"

    def __init__(
        self,
        feed_rate: int = 12000,
        output_width: float = 130,
        output_height=89.375,
        circular_interpolation: bool = False,
    ):
        """Initialize the Svg2gcodeGenerator with the feed rate and seek rate.

        Args:
            feed_rate (int): Feed rate for the CNC machine
            output_width (float): Width of the output image in mm
            output_height (float): Height of the output image in mm
            circular_interpolation (bool): Whether to use circular interpolation
        """
        super().__init__()

        self.feed_rate = feed_rate
        self.output_width = output_width
        self.output_height = output_height
        self.circular_interpolation = circular_interpolation

    def process(self, image_path: Path, output_path: Path) -> None:
        """Process the input svg image and return the output GCode

        Args:
            image (Image): SVG image to be converted to GCode

        Returns: None
        """
        super().process(image_path, output_path)

        # Confirm that the input image exists
        if not image_path.exists():
            raise FileNotFoundError(f"Image {image_path} not found.")

        # Construct the command to convert the SVG to GCode
        command = f"cargo run --manifest-path {self.SVG2GCODE_PATH} --release -- "
        # Add arguments
        command += f"{image_path} --circular-interpolation {"true" if self.circular_interpolation else "false"} -o {output_path} --dimensions {self.output_width}mm,{self.output_height}mm --feedrate {self.feed_rate} --origin 0,0"

        # Run the command
        subprocess.run(command, shell=True)
