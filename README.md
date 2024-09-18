# Etchbot V2

This repo hosts all of the code for Etchbot V2 and Etchbot Stepper.

## Links

[Join our Discord!](https://discord.gg/exDWKb4kwd)

[![Link to YouTube video](https://img.youtube.com/vi/GoBqf2Dr3V8/0.jpg)](https://www.youtube.com/watch?v=GoBqf2Dr3V8)

[![Link to YouTube video](https://img.youtube.com/vi/iQhhutAanu0/0.jpg)](https://www.youtube.com/watch?v=iQhhutAanu0)





## Code Organization

The software for the Etchbot is split into multiple parts:
* `etch-a-sketch-driver`: Contains all of the code for controlling the Etch-a-Sketch (both stepper and brushless varieties).
* `etch-a-sketch-eraser`: Driver for the MotorGo running the etch a sketch eraser.
* `etch-a-sketch-gui`: The web application for uploading images and controlling the Etchbot.
* `etch-a-sketch-server`: Contains all of the code for controlling the Etch-a-Sketch stepper motor.

## Setting up the Etchbot Stepper
### Hardware
All of the CAD files for the Etchbot Stepper can be found at this Printables link.

### Software

The EtchBot Stepper receives GCode commands from the server over WiFi. You will need to run the GUI, server, and flash an ESP32 with the stepper firmware to get the Etchbot Stepper running.

#### Running the GUI and Server
The GUI and server are setup to run with Docker. To run the GUI and server, you will need to have Docker installed on your machine. You can install Docker [here](https://docs.docker.com/get-docker/).

We've set up a bash script to run the GUI and server. To run the GUI and server, run the following command in the root directory of the repository:

```bash
bash start_server.sh
```

#### Flashing the ESP32
You will need to setup PlatformIO to flash the ESP32. You can install PlatformIO [here](https://platformio.org/install/cli).

To flash the ESP32, navigate to the `etch-a-sketch-driver` directory in PlatformIO, switch to the stepper env, and upload the code. Note that you need to update the `ssid` and `password` and the IP address of the etch-a-sketch server in the `main.cpp` file.

