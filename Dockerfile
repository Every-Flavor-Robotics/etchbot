FROM pytorch/pytorch:2.3.0-cuda12.1-cudnn8-runtime

RUN apt update && apt install -y \
    # python3 \
    # python3-pip \
    curl \
    git \
    cmake \
    build-essential \
    libssl-dev \
    pkg-config \
    libudev-dev \
    ffmpeg \
    libsm6 \
    libxext6 \
    build-essential \
    libagg-dev \
    libpotrace-dev \
    pkg-config \
    python-is-python3 \
    unzip \
    libfontconfig1-dev


# Install dependencies using pip and requirements.txt
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# Setup Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain 1.77.0

# Copy the source code into the container
COPY . /app
RUN mkdir /app/pipeline_output
WORKDIR /app

# Run setup script
RUN chmod +x /app/install.sh
RUN ./install.sh

# Update Path to include the rust binaries
ENV PATH="/app/modules/svg2gcode/target/release:${PATH}"


# Run example picture with cli, this will ensure that the models are all downloaded

RUN python3 etch_a_sketch_cli.py -i examples/hot_dog.png -o hot_dog --copy

# Set the default command to run when the container starts
CMD ["python", "etch_a_sketch_server.py"]