For use with Arduino Due

To program the Arduino Due
- you need to make sure you have Arduino Due support in your IDE - follow instructions on https://www.arduino.cc/en/Guide/ArduinoDue
- you need to add the libraries in the arduino libraries folder - follow instructions on https://www.arduino.cc/en/guide/libraries 
- make sure you use the native USB port (for both programming and running the Arduino): https://www.arduino.cc/en/uploads/Guide/DueSerialPorts.jpg 

To use the software
- make sure you have installed pyserial, qtpy, pyqt5, pyqtgraph (you can install these packages with pip3 install. E.g. pip3 install qtpy --user)
- use "sudo usermod -aG $USER dialout" to have access to serial ports without sudo
- use "python3 main.py to launch the program"
