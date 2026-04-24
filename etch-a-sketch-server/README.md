# Etch-A-Sketch Server

This is the server backend for Etchbot.

## Installation
We provide a Dockerfile for easy installation. To build the Docker image, run the following command:
```bash
docker build -t etch-a-sketch .
```

To run the Docker container, run the following command:
```bash
docker run -p 5005:5005 -p 5010:5010 --gpus device=1 etch-a-sketch:latest
```

Note, the `--gpus device=1` flag is optional and is only required if you have a GPU available.
