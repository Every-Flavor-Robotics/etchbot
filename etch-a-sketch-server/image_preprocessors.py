import subprocess
from pathlib import Path
from click import secho
from PIL import Image
from matplotlib.streamplot import OutOfBounds
import numpy as np
import cv2

# Base class for all preprocessors
from preprocessor_utils import Preprocessor

# Processor specific imports
from rembg import remove
from filelock import FileLock, Timeout


class ImagePreprocessor(Preprocessor):
    """Abstract class for image preprocessors.

    All image preprocessors take a Path to an image as an input and return a PIL Image as an output. The process method is
    abstract and must be implemented by the subclass and the output is the image that will be fed to the next step
    in the pipeline.
    """

    SUPPORTED_TYPES = [".png", ".jpg", ".jpeg"]
    OUTPUT_EXTENSION = ".png"


class AspectRatioPreprocessor(ImagePreprocessor):
    """Aspect Ratio Preprocessor crops the input image to a specified size."""

    PARALLELIZABLE = True

    def __init__(self, aspect_ratio: float, resize: bool = False):
        """Initialize the Crop Preprocessor with the crop size.

        Args:
            aspect_ratio (float): Aspect ratio to crop the image to (width / height)
            resize (bool): Whether to resize the image to the crop size
        """
        super().__init__()

        self.aspect_ratio = aspect_ratio
        self.should_resize = resize

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns:
            Path: Path to the processed image, ready for the next step in the pipeline
        """
        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        # Load the image
        image = Image.open(input_path)

        if self.should_resize:
            # Resize the image to the correct aspect ratio
            image = self.resize(image)
        else:
            # Crop the center of the image to the correct aspect ratio
            image = self.crop_center(image)

        while True:
            try:
                with FileLock(output_path.with_suffix(".lock"), timeout=1):
                    # Save the cropped image
                    image.save(output_path)

                break
            except:
                pass

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )

        return output_path

    def compute_new_dimensions(self, width: int, height: int) -> tuple[int, int]:
        """Compute the new dimensions of the image after cropping."""

        # Compute the new dimensions
        new_width = min(width, height * self.aspect_ratio)
        new_height = min(height, width / self.aspect_ratio)

        return int(new_width), int(new_height)

    def crop_center(self, img: Image.Image) -> Image.Image:
        """Crop the center of the image to the specified aspect ratio."""

        # Get the dimensions of the image
        width, height = img.size

        # Compute the new dimensions
        new_width, new_height = self.compute_new_dimensions(width, height)

        # Calculate the dimensions of the center crop
        left = int((width - new_width) / 2)
        top = int((height - new_height) / 2)
        right = int((width + new_width) / 2)
        bottom = int((height + new_height) / 2)

        # Crop the center of the image
        img = img.crop((left, top, right, bottom))

        return img

    def resize(self, img: Image.Image) -> Image.Image:
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

    COLORBOOK_PATH = "./modules/GsColorbook/python-implementation/colorbook_cli.py"
    PARALLELIZABLE = False

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            image (Image): Raw image to be processed

        Returns:
            Path: Path to the processed image, ready for the next step in the pipeline
        """
        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        # Temp directory to store the output of the colorbook preprocessor
        # Remove the image name and extension from the output path
        output_path = output_path.absolute()
        output_temp_dir = output_path.parent / "temp"

        # Construct the command to run the colorbook preprocessor
        command = f"python {self.COLORBOOK_PATH} -i {input_path.absolute()} -o {output_temp_dir}"

        # Run the command
        try:
            subprocess.run(command, check=True, shell=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Preprocessor execution failed: {e}") from e

        # Transfer outlined.png to the output directory
        outlined_path = output_temp_dir / "outlined.png"
        while True:
            try:
                with FileLock(output_path.with_suffix(".lock"), timeout=1):
                    subprocess.run(
                        f"mv {outlined_path} {output_path}", check=True, shell=True
                    )
                    break
            except Timeout:
                pass

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )

        # Remove the temp directory
        subprocess.run(f"rm -r {output_temp_dir}", check=True, shell=True)

        return output_path


class CoherentLineDrawingPreprocessor(ImagePreprocessor):
    """This preprocessor generates a line drawing using the algorithm proposed in "Coherent Line Drawing" (Kang et al)

    Code at: https://github.com/SSARCandy/Coherent-Line-Drawing
    """

    COHERENT_LINE_DRAWING_PATH = "./modules/Coherent-Line-Drawing/build/cld"
    PARALLELIZABLE = False

    def __init__(
        self,
        etf_kernel: int = 5,
        sigma_c: float = 0.361,
        sigma_m: float = 4.0,
        tau: float = 0.9,
        rho: float = 0.997,
        etf_iterations: int = 1,
        cld_iterations: int = 1,
    ):
        """Initialize the Coherent Line Drawing Preprocessor with the parameters.

        Args:
            etf_kernel (int): Size of the edge tangent flow kernel
            sigma_c (float): Line width
            sigma_m (float): Degree of coherence
            tau (float): Thresholding
            rho (float): Noise
        """

        super().__init__()

        self.etf_kernel = etf_kernel
        self.sigma_c = sigma_c
        self.sigma_m = sigma_m
        self.tau = tau
        self.rho = rho
        self.etf_iterations = etf_iterations
        self.cld_iterations = cld_iterations

    def construct_args(self, args: dict) -> str:
        """Construct the arguments for the Coherent Line Drawing preprocessor.

        Args:
            args (dict): Dictionary of arguments for the preprocessor

        Returns:
            str: Arguments for the preprocessor in the correct format
        """

        args_str = " ".join([f"{key} {value}" for key, value in args.items()])

        return args_str

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns: None
        """

        super()._process(input_path, output_path)

        # Construct args
        args = {
            "--ETF_kernel": self.etf_kernel,
            "--sigma_c": self.sigma_c,
            "--sigma_m": self.sigma_m,
            "--tau": self.tau,
            "--rho": self.rho,
            "--ETF_iter": self.etf_iterations,
            "--CLD_iter": self.cld_iterations,
            "--output": output_path,
            "--src": input_path,
        }

        # Construct the command to run the Coherent Line Drawing preprocessor
        command = f"{self.COHERENT_LINE_DRAWING_PATH} {self.construct_args(args)}"

        # Run the command
        while True:
            try:
                with FileLock(output_path.with_suffix(".lock"), timeout=1):
                    try:
                        subprocess.run(command, check=True, shell=True)
                    except subprocess.CalledProcessError as e:
                        raise RuntimeError(f"Preprocessor execution failed: {e}") from e

                    # Confirm that the output image exists
                    if not output_path.exists():
                        raise FileNotFoundError(
                            f"Output image {output_path} not generated correctly."
                        )

                    # Open the image and convert to grayscale
                    image = Image.open(output_path).convert("L")
                    image.save(output_path)

                break
            except:
                pass

        return output_path


class DeNoisePreprocessor(ImagePreprocessor):
    """DeNoise Preprocessor removes noise from the input image."""

    PARALLELIZABLE = True

    def __init__(self, noise_factor: float = 0.5):
        """Initialize the DeNoise Preprocessor with the noise factor.

        Args:
            noise_factor (float): Factor to remove noise from the image
        """
        super().__init__()

        self.noise_factor = noise_factor

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns: None
        """

        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        # Load the image
        image = Image.open(input_path)

        # Remove noise from the image
        image = self.de_noise(image)

        while True:
            try:
                with FileLock(output_path.with_suffix(".lock"), timeout=1):
                    # Save the de-noised image
                    image.save(output_path)

                    break
            except:
                pass

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )

        return output_path

    def de_noise(self, img: Image.Image) -> Image.Image:
        """Remove noise from the image."""
        # Convert PIL image to numpy array
        img_array = np.array(img)

        # Apply Non-Local Means Denoising algorithm
        denoised_img_array = cv2.fastNlMeansDenoisingColored(
            img_array, None, self.noise_factor * 10, self.noise_factor * 10, 7, 21
        )

        # Convert numpy array back to PIL image
        denoised_img = Image.fromarray(denoised_img_array)

        return denoised_img


class BlackAndWhitePreprocessor(ImagePreprocessor):
    """Black and White Preprocessor converts the input image to black and white."""

    PARALLELIZABLE = False

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns: None
        """

        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        # Load the image
        image = Image.open(input_path)

        # Convert the image to black and white
        image = image.convert("L")

        while True:
            try:
                with FileLock(output_path.with_suffix(".lock"), timeout=1):
                    # Save the black and white image
                    image.save(output_path)

                    break
            except:
                pass

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )

        return output_path


class RemBGPreprocessor(ImagePreprocessor):
    """RemBG Preprocessor removes the background from the input image."""

    PATH_TO_BACKGROUND_REMOVER = "backgroundremover"
    PARALLELIZABLE = True

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns: None
        """

        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        command = f"{self.PATH_TO_BACKGROUND_REMOVER} -i {input_path} -o {output_path}"

        # Run the command
        try:
            subprocess.run(command, check=True, shell=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Preprocessor execution failed: {e}") from e

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )

        return output_path


class CartoonifyPreProcessor(ImagePreprocessor):
    """Cartoonify Preprocessor converts the input image to a cartoon-like image.

    https://github.com/ahmedbesbes/cartoonify

    """

    PATH_TO_CARTOONIFY = "./modules/cartoonify/cartoongan/etch_a_sketch_demo.py"
    PARALLELIZABLE = False

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns: None
        """

        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        args = f"--input_path {input_path} --output_path {output_path}"

        # Construct the command to run the Cartoonify preprocessor
        command = f"python {self.PATH_TO_CARTOONIFY} {args}"

        # Run the command
        try:
            subprocess.run(command, check=True, shell=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Preprocessor execution failed: {e}") from e

        # Confirm that the output image exists
        if not output_path.exists():
            raise FileNotFoundError(
                f"Output image {output_path} not generated correctly."
            )

        return output_path


class InformativeDrawingsPreprocessor(ImagePreprocessor):
    """Informative Drawings Preprocessor generates informative drawings from the input image."""

    PATH_TO_INFORMATIVE_DRAWINGS = "./modules/informative-drawings/run_etch.py"
    PARALLELIZABLE = True

    def _process(self, input_path: Path, output_path: Path) -> Path:
        """Process the input image and return the output image.

        Args:
            input_path (Path): Path to the raw image to be processed
            output_path (Path): Path to save the processed image

        Returns: None
        """

        super()._process(input_path, output_path)

        # Confirm that the input image exists
        if not input_path.exists():
            raise FileNotFoundError(f"Image {input_path} not found.")

        args = (
            f"--input_path {input_path} --output_path {output_path} --name anime_style"
        )

        # Construct the command to run the Informative Drawings preprocessor
        command = f"python {self.PATH_TO_INFORMATIVE_DRAWINGS} {args}"

        while True:
            try:
                with FileLock(output_path.with_suffix(".lock"), timeout=1):
                    # Run the command
                    try:
                        subprocess.run(command, check=True, shell=True)
                    except subprocess.CalledProcessError as e:
                        raise RuntimeError(f"Preprocessor execution failed: {e}") from e

                    # Confirm that the output image exists
                    if not output_path.exists():
                        raise FileNotFoundError(
                            f"Output image {output_path} not generated correctly."
                        )

                    break
            except:
                pass

        return output_path
