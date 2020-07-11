# pufferfish-prototype-v0

This repository contains the software source and hardware design files for the v0 prototype of the Pufferfish ventilator. Note that this prototype version does not have a uniform or high level of documentation. We are working to prepare the software and hardware files for the v1 prototype for open-source release. The v1 prototype uses an STM32 microcontroller and a Raspberry Pi 4, while the v0 prototype uses an Arduino Due microcontroller board and a Jetson Nano. The v0 prototype software is for development purposes and supports logging sensor readings and other variables at 667 Hz (limited by the 1.5 ms cycle for reading all 8 I2C sensors currently in use).


## Usage

### Software

![Screenshot](/dev\ interface.png)

To program the Arduino Due
- you need to make sure you have Arduino Due support in your IDE - follow instructions on https://www.arduino.cc/en/Guide/ArduinoDue
- you need to add the libraries in the arduino libraries folder - follow instructions on https://www.arduino.cc/en/guide/libraries 
- make sure you use the native USB port (for both programming and running the Arduino): https://www.arduino.cc/en/uploads/Guide/DueSerialPorts.jpg 

To use the software
- make sure you have installed pyserial, qtpy, pyqt5, pyqtgraph (you can install these packages with pip3 install. E.g. pip3 install qtpy --user)
- use "sudo usermod -aG $USER dialout" to have access to serial ports without sudo
- use "python3 main.py to launch the program"

Notion page for more technical details including setting up the hardware: https://www.notion.so/pufferfish/Pez-Globo-2bedf862f80645cc90a4c7d114437cf6 


## Licensing

All contents of this repository, except where otherwise indicated, are released under the Permissive License Agreement in `LICENSE.pdf`.

Due to licensing requirements of an Arduino library we are using, the `controller/controller.ino` file is licensed under GPL v3. A copy of the GPL v3 license can be found at `controller/LICENSE.GPL`. Note that, except where otherwise indicated, all other files in the `controller` directory are still licensed under our Permissive License Agreement at `LICENSE.pdf`.
