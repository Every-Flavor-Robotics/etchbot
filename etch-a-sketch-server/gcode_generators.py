import subprocess
from pathlib import Path

# Base class for all preprocessors
from preprocessor_utils import Preprocessor


class GCodeGenerator(Preprocessor):
    """Abstract class for GCode Generators.

    All GCode Generators take a Path to an image as an input and return a Path to a GCode file as an output. The process
    method is abstract and must be implemented by the subclass and the output is the GCode file that will be fed to the
    next step in the pipeline.
    """

    SUPPORTED_TYPES = [".svg"]
    OUTPUT_EXTENSION = ".gcode"


class Svg2GcodeGenerator(GCodeGenerator):
    """Svg2gcodeGenerator uses svg2gcode to convert the input SVG to GCode.

    https://github.com/sameer/svg2gcode.git
    """

    SVG2GCODE_PATH = "./modules/svg2gcode/Cargo.toml"
    PARALLELIZABLE = True

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

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input svg image and return the output GCode

        Args:
            image (Image): SVG image to be converted to GCode

        Returns: None
        """
        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        # Construct the command to convert the SVG to GCode
        command = f"{Svg2GcodeGenerator.SVG2GCODE_PATH} "
        # Add arguments
        command += f"{input_path} --circular-interpolation {'true' if self.circular_interpolation else 'false'} -o {output_path} --dimensions {self.output_width}mm,{self.output_height}mm --feedrate {self.feed_rate} --origin 0,0"

        # Run the command
        subprocess.run(command, shell=True)

        return output_path
