For use with Arduino Due

Before running the Arduino program
- you need to make sure you have Arduino Due support in your IDE - follow instructions on https://www.arduino.cc/en/Guide/ArduinoDue
- you need to add the libraries in the sensor libraries folder - follow instructions on https://www.arduino.cc/en/guide/libraries (Importing a .zip Library)
- make sure you use the native USB port (for both programming and running the Arduino): https://www.arduino.cc/en/uploads/Guide/DueSerialPorts.jpg 

To use the software
- make sure you have installed pyserial, qtpy, pyqt5, pyqtgraph (you can install these packages with pip3 install. E.g. pip3 install qtpy --user)
- use "sudo usermod -aG $USER dialout" to have access to serial ports without sudo
- use "python3 main.py to launch the program"

Folder structure
- pressure drop vs flow measurement with stepper control can be used to move the linear actuator (including closing the valve by stopping the flow), and log stepper position, pressure and flow. Currently included are three Honeywell HSC pressure sensors and one Sensirion SFM3000/SFM3200 flow sensor. Contact Hongquan for questions.
- actuator_speed_test actuates the valve at > 360 cycles per minute, data logging is currently done in a separate program (evaluation software for SFM3300-AW) but will be added to the firmware/python program. Contact Hongquan for questions
- valve characterization_with encoder uses a linear magnetic encoder to monitor the position of the pinch shoe. Contact Deepak for questions.
