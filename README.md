# etch-a-sketch-server
This repo hosts all of the software for the Etch-a-Sketch Camera! Check out the video here:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/iQhhutAanu0/0.jpg)](https://www.youtube.com/watch?v=iQhhutAanu0)

This robot is capable of taking an image with the onboard camera, converting the image into GCode commands, and executing the GCode to draw the image on the screen. 

# Organization
There are two main components to this project, the image to gcode conversion, and the microncontroller/motor driver firmware.

## Image to GCode Conversion
The main entry point for the code is `etch_a_sketch_server_cli.py`. The CLI chains together various processing steps to convert the image into GCode. The final output of this pipeline is a GCode file. `gcode_server.py` handles streaming the GCode to the robot over Wifi. 

More details about the details of the pipeline to come in the README

## Motor Driver Software
The robot controls the motor using a MotorGo motor controller, our in house motor controller with an ESP32 on board (more info [here](motorgo.net)). The code was written using PlatformIO and is located under `etch_a_sketch_driver`. The main entry point for this code is `src/main.cpp`

# Hardware
All of the 3D printable files are availabe on [Printables](https://www.printables.com/model/884473-every-flavor-robotics-etch-a-sketch-camera). The MotorGo, the motor controller onboard the robot, can be purchased at [motorgo.net](motorgo.net). Full BOM coming soon!

# Important Links

Here are the links to the open-source projects this codebase relies on.
* [svg2gcode](https://github.com/sameer/svg2gcode)
* [cartoonify](https://github.com/ahmedbesbes/cartoonify) | [Our Fork](https://github.com/Every-Flavor-Robotics/cartoonify)
* [gcode-optimizer](https://github.com/andrewhodel/gcode-optimizer) | [Our Fork](https://github.com/Every-Flavor-Robotics/gcode-optimizer)
* [rembg](https://github.com/danielgatis/rembg)
* [potrace](https://potrace.sourceforge.net) | [GitHub mirror](https://github.com/skyrpex/potrace)
* [ColoringPags](https://github.com/eurashin/ColoringPags)
* [Coherent-Line-Drawing](https://github.com/SSARCandy/Coherent-Line-Drawing)

# Supporting Us
* Subscribe to our [YouTube Channel](https://www.youtube.com/@EveryFlavorRobot)
* Join our [Discord](https://discord.gg/exDWKb4kwd) if you have questions or just want to chat!
* Consider supporting us on [Patreon](https://www.patreon.com/EveryFlavorRobot)
