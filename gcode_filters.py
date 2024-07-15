import subprocess
from abc import ABC, abstractmethod
from pathlib import Path
import click

# Base class for all preprocessors
from preprocessor_utils import Preprocessor


class GCodeFilter(Preprocessor):
    """Abstract class for GCode Filters.

    All GCode Filters take a Path to a GCode file as an input and return a Path to a GCode file as an output. The process
    method is abstract and must be implemented by the subclass and the output is the GCode file that will be fed to the
    next step in the pipeline.
    """

    SUPPORTED_TYPES = [".gcode"]
    OUTPUT_EXTENSION = ".gcode"

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
        # Remove comments signified by a parenthesis
        line = line.split("(")[0]

        parts = line.split(" ")
        command = parts[0]
        args = {}

        for part in parts[1:]:
            key = part[0]
            try:
                value = float(part[1:])
            except:
                breakpoint()
            args[key] = value
        return command, args


class GCodeCleaner(GCodeFilter):
    """GCodeCleaner removes comments and empty lines from the GCode file."""

    PARALLELIZABLE = True

    def _process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with comments and empty lines removed.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super()._process(gcode_path, output_path)

        with open(gcode_path, "r") as f:
            lines = f.readlines()

        new_lines = []
        for line in lines:
            line = line.split(";")[0]
            line = line.split("(")[0]
            line = line.split("%")[0]
            line = line.strip()
            if line != "" and not line.startswith(";"):
                new_lines.append(line + "\n")

        with open(output_path, "w") as f:
            f.writelines(new_lines)


class RemoveZ(GCodeFilter):
    """RemoveZ removes the Z axis from the GCode file.

    If a command only has a Z axis, it will be removed completely. If a command has an X, Y, and Z axis, the Z axis
    will be removed.

    """

    PARALLELIZABLE = True

    def _process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with the Z axis removed.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super()._process(gcode_path, output_path)

        with open(gcode_path, "r") as f:
            lines = f.readlines()

        new_lines = []
        for line in lines:
            command, args = self.decode_line(line)

            if "Z" in args:
                del args["Z"]
                # Remove F it is the only argument after Z
                if "F" in args and len(args) == 1:
                    del args["F"]

            if len(args) > 0:
                new_line = f"{command} "
                for key, value in args.items():
                    new_line += f"{key}{value} "

                new_line = new_line.strip() + "\n"
                new_lines.append(new_line)

        with open(output_path, "w") as f:
            f.writelines(new_lines)


class LoopCloser(GCodeFilter):
    """LoopCloser closes loops in the GCode file.

    The LoopCloser checks the first and last points in the GCode file and adds a line between them if they are not the
    same.

    """

    PARALLELIZABLE = True

    def _process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with closed loops.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super()._process(gcode_path, output_path)

        with open(gcode_path, "r") as f:
            lines = f.readlines()

        if len(lines) == 0:
            return

        first_line = lines[0]
        last_line = lines[-1]

        first_command, first_args = self.decode_line(first_line)
        last_command, last_args = self.decode_line(last_line)

        first_x = first_args["X"]
        first_y = first_args["Y"]

        last_x = last_args["X"]
        last_y = last_args["Y"]

        if first_x != last_x or first_y != last_y:
            # Add a line to close the loop
            lines.append(f"G1 X{first_x} Y{first_y}\n")

        with open(output_path, "w") as f:
            f.writelines(lines)


class ResolutionReducer(GCodeFilter):
    """ResolutionReducer reduces the resolution of the GCode by removing points that fall within a certain distance of each other.

    Ignores the z axis and only considers the x and y axes.

    """

    PARALLELIZABLE = True

    def __init__(self, tolerance: float = 0.1):
        """Initialize the ResolutionReducer with the tolerance.

        Args:
            tolerance (float): Distance tolerance for removing points in mm
        """
        super().__init__()

        self.tolerance = tolerance

    def _process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with reduced resolution.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super()._process(gcode_path, output_path)

        with open(gcode_path, "r") as f:
            lines = f.readlines()

        new_lines = []
        last_point = None

        previous_g0 = None
        for i, line in enumerate(lines):
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
            elif command == "G0":
                if previous_g0 is not None:
                    new_lines.append(previous_g0)
                previous_g0 = None

                new_lines.append(line)

                x = args["X"]
                y = args["Y"]
                # Loop to find the next G0
                for j in range(i + 1, len(lines)):
                    next_command, next_args = self.decode_line(lines[j])
                    if next_command == "G0":
                        break

                command, args = self.decode_line(lines[j - 1])
                if command == "G1":
                    x_end = args["X"]
                    y_end = args["Y"]

                    # Compare the distance between the last point and the end point
                    distance = ((x - x_end) ** 2 + (y - y_end) ** 2) ** 0.5
                    if distance < self.tolerance:
                        previous_g0 = f"G1 X{x_end} Y{y_end}\n"

            else:
                new_lines.append(line)

        with open(output_path, "w") as f:
            f.writelines(new_lines)


class ColinearFilter(GCodeFilter):
    """ColinearFilter removes colinear points from the GCode.

    Ignores the z axis and only considers the x and y axes.

    """

    PARALLELIZABLE = True

    def __init__(self, dot_product_threshold: float = 0.996):
        """Initialize the ColinearFilter."""
        super().__init__()

        self.dot_product_threshold = dot_product_threshold

    def _process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with colinear points removed.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super()._process(gcode_path, output_path)

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

        try:
            # Calculate the dot product
            dot_product = (v1[0] * v2[0] + v1[1] * v2[1]) / (mag1 * mag2)
        except:
            dot_product = 0
        return dot_product


class TSPOptimizer(GCodeFilter):
    """TSPOptimizer optimizes the GCode file using the Travelling Salesman Problem.

    The TSPOptimizer uses the Concorde TSP solver to optimize the order of the points in the GCode file. The Concorde
    solver must be installed on the system and available in the PATH.
    """

    OPTIMIZER_PATH = "~/efr/gcode-optimizer/rust_optimizer.py"
    PARALLELIZABLE = True

    def __init__(self):
        """Initialize the TSPOptimizer."""
        super().__init__()

    def _process(self, gcode_path: Path, output_path: Path) -> None:
        """Process the input gcode file and return the output gcode file with the points optimized.

        Args:
            gcode_path (Path): Path to the GCode file to be processed
            output_path (Path): Path to save the processed GCode file

        Returns:
            Path: Path to the processed GCode file, ready for the next step in the pipeline
        """
        super()._process(gcode_path, output_path)

        # Construct the command to run the optimizer
        command = f"python {self.OPTIMIZER_PATH} --input_file {gcode_path} --output_file {output_path}"

        # Run the optimizer
        subprocess.run(command, shell=True)
