import os
from abc import ABC, abstractmethod
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

from click import secho


class Preprocessor(ABC):
    """Abstract class for all preprocessors

    A preprocessor is a class that takes in input data, processes it, and writes it to an output file. The input data can be a single file or a directory of files. The output data can be a single file or a directory of files.
    """

    SUPPORTED_TYPES = []
    OUTPUT_EXTENSION = ""
    PARALLELIZABLE = False

    def __init__(self):
        # Confirm that SUPPORTED_TYPES is a list and longer than 0
        if not isinstance(self.SUPPORTED_TYPES, list) or len(self.SUPPORTED_TYPES) == 0:
            raise ValueError(
                "SUPPORTED_TYPES must be a list of supported file types (e.g., ['.png', '.jpg', '.jpeg'])."
            )

        # Confirm that OUTPUT_EXTENSION is a string
        if not isinstance(self.OUTPUT_EXTENSION, str):
            raise ValueError("OUTPUT_EXTENSION must be a string.")

        if not isinstance(self.PARALLELIZABLE, bool):
            raise ValueError("PARALLELIZABLE must be a boolean.")

    def process(self, input_path: Path, output_dir: Path, output_name: str) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the input file to process
            output_dir (Path): Path to the directory to save the processed data
            output_name (str): Name of the output file or directory

        Returns: None
        """

        # Print class name and "Running..." in green
        secho(f"\tRunning {self.__class__.__name__}...", fg="green")

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        output_path = None
        # Check if batch or if single image
        if self._is_batch(input_path):
            # Process the batch of images
            output_path = self._process_batch(input_path, output_dir, output_name)

            # Confirm that the output directory exists
            if not output_path.exists():
                raise FileNotFoundError(
                    f"Output directory {output_path} not generated correctly."
                )
        else:
            # Process the single image
            output_path = output_dir / self._add_output_file_extension(output_name)
            output_path = self._process(input_path, output_path)

            if not output_path.exists():
                raise FileNotFoundError(
                    f"Output image {output_path} not generated correctly."
                )

        return output_path

    @abstractmethod
    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input file and return the output file.

        Args:
            input_path (Path): Path to the input file to process
            output_path (Path): Path to save the processed image

        Returns:
            Path: Path to the processed file, ready for the next step in the pipeline
        """

    def _process_batch(
        self, input_dir: Path, output_dir: Path, output_name: str
    ) -> Path:
        """Process a batch of input files and return the output files.

        Args:
            image_dir (Path): Path to the directory containing the input files
            output_dir (Path): Path to the directory to save the processed output files

        Returns:
            Path: Path to the directory containing the processed files, ready for the next step in the pipeline
        """

        # Get a list of all input files of SUPPORTED_TYPES
        input_files = self._get_supported_files(input_dir)

        # Create the output directory
        output_dir = output_dir / output_name
        output_dir.mkdir(parents=True, exist_ok=True)

        # Create a function to process each input file
        def process_file(input_file: Path):
            output_path = output_dir / self._add_output_file_extension(input_file.stem)
            self._process(input_file, output_path)

        # Process each input
        # if PARALLELIZABLE is True, use ThreadPoolExecutor
        if self.PARALLELIZABLE:
            with ThreadPoolExecutor() as executor:
                executor.map(process_file, input_files)
        else:
            for input_file in input_files:
                process_file(input_file)

        return output_dir

    def _is_batch(self, path: Path) -> bool:
        """Check if the input path is a directory.

        Args:
            path (Path): Path to check

        Returns:
            bool: True if the path is a directory, False otherwise
        """
        return path.is_dir()

    def _get_supported_files(self, input_dir: Path) -> list[Path]:
        """Get a list of all input files of SUPPORTED_TYPES

        Args:
            input_dir (Path): Path to the directory containing the input files

        Returns:
            list: List of all input files of SUPPORTED_TYPES
        """

        # Get a list of all input files of SUPPORTED_TYPES
        input_files = []
        for file_type in self.SUPPORTED_TYPES:
            input_files.extend(input_dir.glob(f"*{file_type}"))

        return input_files

    def _add_output_file_extension(self, output_path: str) -> Path:
        """Add the correct file extension to the output path.

        Args:
            output_path (Path): Path to the output file

        Returns:
            The output path with the correct file extension

        """

        # Add the correct file extension to the output path
        return Path(f"{output_path}{self.OUTPUT_EXTENSION}")
