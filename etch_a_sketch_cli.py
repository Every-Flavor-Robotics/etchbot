import pathlib

import click

from image_preprocessors import ColorbookPreprocessor, AspectRatioPreprocessor
from vectorizers import PotraceVectorizer
from gcode_generators import Svg2GcodeGenerator

def run_pipeline(input_file: pathlib.Path, output_dir: pathlib.Path):
    # Print in green, processing the input file
    click.secho(f"Processing {input_file}...", fg="green")

    # Confirm the output directory exists
    if not output_dir.exists():
        output_dir.mkdir()

    # Create the strategies for each of the steps
    preprocessors = [AspectRatioPreprocessor(16/11), ColorbookPreprocessor()]
    vectorizers = [PotraceVectorizer()]
    gcode_converters = [Svg2GcodeGenerator()]

    # Confirm that the input file exists
    if not input_file.exists():
        raise FileNotFoundError(f"Input file {input_file} not found.")

    # Copy to the output directory
    input_file = input_file.resolve()
    new_input_file = output_dir / input_file.name
    input_file.rename(new_input_file)
    input_file = new_input_file


    click.secho(f"Pre-Processing...", fg="green")

    # Process the input file
    for preprocessor in preprocessors:
        preprocessor.process(input_file, output_dir / "preprocessed.png")
        input_file = output_dir / "preprocessed.png"

    # Print in green, processing complete
    click.secho(f"Pre-Processing complete!", fg="green")

    click.secho(f"Vectorizing...", fg="green")

    for vectorizer in vectorizers:
        # Process the preprocessed image
        vectorizer.process(input_file, output_dir / "vectorized.svg")
        input_file = output_dir / "vectorized.svg"

    # Print in green, processing complete
    click.secho(f"Vectorizing complete!", fg="green")

    # Convert to G-code
    click.secho(f"Converting to G-code...", fg="green")
    for gcode_converter in gcode_converters:
        gcode_converter.process(input_file, output_dir / "output.gcode")

    return output_dir / "output.gcode"



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
def main(input_file: click.Path, output_dir: click.Path):
    run_pipeline(input_file, output_dir)



if __name__ == "__main__":
    main()
