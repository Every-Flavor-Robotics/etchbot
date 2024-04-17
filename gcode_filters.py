import subprocess
from abc import ABC, abstractmethod
from pathlib import Path
import click


class GCodeFilter(ABC):
    """Abstract class for GCode Filters.

    All GCode Filters take a Path to a GCode file as an input and return a Path to a GCode file as an output. The process
    method is abstract and must be implemented by the subclass and the output is the GCode file that will be fed to the
    next step in the pipeline.
    """

    def __init__(self):
        pass

    @abstractmethod
    def process(self, gcode_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed gcode

        Returns:
            Path: Path to the gcode image, ready for the next step in the pipeline
        """

        # Print class name and "Running..." in green
        click.secho(f"\tRunning {self.__class__.__name__}...", fg="green")

    def decode_line(self, line: str) -> tuple[str, dict]:
        """Decode a line from the GCode file.

        Args:
            line (str): Line from the GCode file

        Returns:
            str: Command
            dict: Arguments
        """
        # First, remove comments, signified by a semicolon
        line = line.split(";")[0]
        parts = line.split(" ")
        command = parts[0]
        args = {}
        for part in parts[1:]:
            key = part[0]
            value = float(part[1:])
            args[key] = value
        return command, args


class ResolutionReducer(GCodeFilter):
    """ResolutionReducer reduces the resolution of the GCode by removing points that fall within a certain distance of each other.

    Ignores the z axis and only considers the x and y axes.

    """

    def __init__(self, tolerance: float = 0.1):
        """Initialize the ResolutionReducer with the tolerance.

        Args:
            tolerance (float): Distance tolerance for removing points in mm
        """
        super().__init__()

        self.tolerance = tolerance

    def process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with reduced resolution.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super().process(gcode_path, output_path)

        with open(gcode_path, "r") as f:
            lines = f.readlines()

        new_lines = []
        last_point = None

        i = 0
        for line in lines:
            command, args = self.decode_line(line)

            if command == "G1":
                x = args["X"]
                y = args["Y"]

                if last_point is None:
                    last_point = (x, y)
                    new_lines.append(line)
                else:
                    last_x, last_y = last_point
                    distance = ((x - last_x) ** 2 + (y - last_y) ** 2) ** 0.5
                    if distance > self.tolerance:
                        new_lines.append(line)
                        last_point = (x, y)
            else:
                new_lines.append(line)

        with open(output_path, "w") as f:
            f.writelines(new_lines)


class ColinearFilter(GCodeFilter):
    """ColinearFilter removes colinear points from the GCode.

    Ignores the z axis and only considers the x and y axes.

    """

    def __init__(self, dot_product_threshold: float = 0.996):
        """Initialize the ColinearFilter."""
        super().__init__()

        self.dot_product_threshold = dot_product_threshold

    def process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with colinear points removed.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super().process(gcode_path, output_path)

        with open(gcode_path, "r") as f:
            lines = f.readlines()

        starting_length = len(lines)

        new_lines = []
        last_point = None
        middle_point = None
        for i, line in enumerate(lines):
            command, args = self.decode_line(line)

            # Add all non-G1 commands to the new lines
            if not command == "G1":
                new_lines.append(line)
                # Reset the last point
                last_point = None
                middle_point = None
                continue

            # New starting point for colinear check
            if last_point is None:
                last_point = (args["X"], args["Y"])
                new_lines.append(line)
            # New middle point for colinear check
            elif middle_point is None:
                middle_point = (args["X"], args["Y"])
                new_lines.append(line)
                continue
            # New ending point for colinear check
            else:
                end_point = (args["X"], args["Y"])
                # Calculate the dot product of the vectors
                dot_product = self.calculate_nomralized_dot_product(
                    last_point, middle_point, end_point
                )

                # If the dot product is greater than the threshold, remove the middle point
                if dot_product > self.dot_product_threshold:
                    new_lines.pop()
                    # Keep last point, but update the middle point to end point
                    middle_point = end_point
                else:
                    # If the dot product is less, then shift all points over
                    last_point = middle_point
                    middle_point = end_point
                new_lines.append(line)

        with open(output_path, "w") as f:
            f.writelines(new_lines)

        ending_length = len(new_lines)
        print(f"Removed {starting_length - ending_length} colinear points.")

    def calculate_nomralized_dot_product(self, p1, p2, p3):
        """Calculate the normalized dot product. Equal to the cosine of the angle between the two vectors.

        Args:
            p1 (tuple): First point (x, y)
            p2 (tuple): Second point (x, y)
            p3 (tuple): Third point (x, y)

        Returns:
            float Dot product of the vectors
        """
        # Calculate the vectors
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])

        # Calculate the magnitudes
        mag1 = (v1[0] ** 2 + v1[1] ** 2) ** 0.5
        mag2 = (v2[0] ** 2 + v2[1] ** 2) ** 0.5

        # Calculate the dot product
        dot_product = (v1[0] * v2[0] + v1[1] * v2[1]) / (mag1 * mag2)

        return dot_product
