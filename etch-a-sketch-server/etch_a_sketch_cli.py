from genericpath import exists
import numpy as np
import os
from config import Config

os.environ["MKL_THREADING_LAYER"] = "INTEL"
os.environ["MKL_SERVICE_FORCE_INTEL"] = "1"

import pathlib
import shutil
import click
import time

from video_preprocessors import FFmpegSplitter

from image_preprocessors import (
    ColorbookPreprocessor,
    AspectRatioPreprocessor,
    CoherentLineDrawingPreprocessor,
    BlackAndWhitePreprocessor,
    RemBGPreprocessor,
    CartoonifyPreProcessor,
    InformativeDrawingsPreprocessor,
)
from vectorizers import PotraceVectorizer
from gcode_generators import Svg2GcodeGenerator
import shutil
from gcode_filters import (
    ResolutionReducer,
    ColinearFilter,
    TSPOptimizer,
    GCodeCleaner,
    RemoveZ,
)

counter = 0


def copy_files(source, destination):
    if source.is_file():
        shutil.copy(source, destination)
    elif source.is_dir():
        shutil.copytree(source, destination, dirs_exist_ok=True)


def file_type_correct(path: pathlib.Path, file_type: list[str]):
    # If directory, check if any file in the directory is of the correct type
    if path.is_dir():
        for file in path.iterdir():
            if file.suffix in file_type:
                return True
        return False
    # If file, check if the file is of the correct type
    elif path.is_file():
        return path.suffix in file_type
    return False


def run_pipeline(
    input_file: pathlib.Path,
    output_dir: pathlib.Path,
    copy: bool = False,
    skip_preprocessing: bool = False,
):
    global counter
    # Print in green, processing the input file
    click.secho(f"Processing {input_file}...", fg="green")

    # Confirm the output directory exists
    if not output_dir.exists():
        output_dir.mkdir()

        # Create the strategies for each of the steps
        # preprocessors = [
        #     # AspectRatioPreprocessor(16 / 11),
        #     RemBGPreprocessor(),
        #     CoherentLineDrawingPreprocessor(
        #         etf_kernel=5, sigma_c=1.025, sigma_m=4.732, tau=0.894, rho=0.994
        #     ),
        #     BlackAndWhitePreprocessor(),
        # ]

    video_splitters = [FFmpegSplitter(4)]

    preprocessors = [
        RemBGPreprocessor(),
        AspectRatioPreprocessor(16 / 11),
        # CartoonifyPreProcessor(),
        InformativeDrawingsPreprocessor(),
        # ColorbookPreprocessor(),
        BlackAndWhitePreprocessor(),
    ]
    vectorizers = [PotraceVectorizer()]
    gcode_converters = [Svg2GcodeGenerator(feed_rate=Config().get("drawing.feed_rate", None),
                output_width=Config().get("drawing.width", None),
                output_height=Config().get("drawing.height", None))]
    gcode_filters = [
        GCodeCleaner(),
        RemoveZ(),
        ResolutionReducer(1.2),
        ColinearFilter(0.996),
        TSPOptimizer(),
    ]

    # Confirm that the input file exists
    if not input_file.exists():
        raise FileNotFoundError(f"Input file {input_file} not found.")

    # If copy is True, copy the input file to the output directory
    # otherwise, move the input file to the output directory
    if copy:
        input_file = input_file.resolve()
        new_input_file = output_dir / input_file.name

        copy_files(input_file, new_input_file)

        input_file = new_input_file
    else:
        input_file = input_file.resolve()
        new_input_file = output_dir / input_file.name
        input_file.rename(new_input_file)
        input_file = new_input_file

    if not file_type_correct(input_file, [".mp4", ".mov", ".avi", ".mkv"]):
        click.secho(f"Skipping video splitting...", fg="red")

    else:
        click.secho(f"Splitting video...", fg="green")

        for video_splitter in video_splitters:
            input_file = video_splitter.process(input_file, output_dir, "split")

            copy_files(
                input_file,
                output_dir
                / f"step_{counter}_preprocessed_{type(video_splitter).__name__}",
            )

            counter += 1

        click.secho(f"Splitting video complete!", fg="green")

    # Only run preprocessor if input file is an image
    if (
        not file_type_correct(input_file, [".jpg", ".jpeg", ".png"])
        or skip_preprocessing
    ):
        # Print in red, skipping pre-processing
        click.secho(f"Skipping pre-processing...", fg="red")
    else:
        click.secho(f"Pre-Processing...", fg="green")

        # Process the input file
        for preprocessor in preprocessors:
            input_file = preprocessor.process(input_file, output_dir, "preprocessed")

            # Create a copy of the preprocessed image
            copy_files(
                input_file,
                output_dir
                / f"step_{counter}_preprocessed_{type(preprocessor).__name__}.png",
            )
            counter += 1
            time.sleep(0.1)

        # Print in green, processing complete
        click.secho(f"Pre-Processing complete!", fg="green")
    if not file_type_correct(input_file, [".jpg", ".jpeg", ".png"]):
        # Print in red, skipping pre-processing
        click.secho(f"Skipping Vectorizing...", fg="red")
    else:
        click.secho(f"Vectorizing...", fg="green")

        for vectorizer in vectorizers:
            # Process the preprocessed image
            input_file = vectorizer.process(input_file, output_dir, "vectorized")

            copy_files(
                input_file,
                output_dir
                / f"step_{counter}_preprocessed_{type(vectorizer).__name__}.svg",
            )
            counter += 1

        # Print in green, processing complete
        click.secho(f"Vectorizing complete!", fg="green")

    if not file_type_correct(input_file, [".svg"]):
        # Print in red, skipping pre-processing
        click.secho(f"Skipping g-code conversion...", fg="red")
    else:
        # Convert to G-code
        click.secho(f"Converting to G-code...", fg="green")
        for gcode_converter in gcode_converters:
            input_file = gcode_converter.process(
                input_file, output_dir, "unprocessed_gcode"
            )

            copy_files(
                input_file,
                output_dir
                / f"step_{counter}_preprocessed_{type(gcode_converter).__name__}.gcode",
            )

            counter += 1

        # Print in green, processing complete
        click.secho(f"Converting to G-code complete!", fg="green")

    if not file_type_correct(input_file, [".gcode"]):
        # Print in red, skipping pre-processing
        click.secho(f"Skipping G-code filtering...", fg="red")
    else:
        # Filter the G-code
        click.secho(f"Filtering G-code...", fg="green")
        for gcode_filter in gcode_filters:
            input_file = gcode_filter.process(input_file, output_dir, "output")

            copy_files(
                input_file,
                output_dir
                / f"step_{counter}_preprocessed_{type(gcode_filter).__name__}.optgcode",
            )

            counter += 1

    return input_file


@click.command()
@click.option(
    "--input-file",
    "-i",
    help="Input file",
    required=True,
    type=click.Path(exists=True, path_type=pathlib.Path),
)
@click.option(
    "--output-dir",
    "-o",
    help="Output path",
    required=True,
    type=click.Path(file_okay=False, path_type=pathlib.Path),
)
@click.option(
    "--copy",
    "-c",
    help="Copy the input file to the output directory",
    is_flag=True,
)
@click.option(
    "--skip-preprocessing",
    "-s",
    help="Skip pre-processing",
    is_flag=True,
)
def main(
    input_file: pathlib.Path,
    output_dir: pathlib.Path,
    copy: bool,
    skip_preprocessing: bool,
):
    run_pipeline(input_file, "pipeline_output" / output_dir, copy, skip_preprocessing)


if __name__ == "__main__":
    main()
