#include <Wire.h>

#include <sdpsensor.h>

SDP8XXSensor sdp;

void setup() {
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established
  
  Wire.begin();
  sdp.stopContinuous(); // stop continuous measurement if it's running
  do {
    int ret = sdp.init();
    if (ret == 0) {
      SerialUSB.print("init(): success\n");
      break;
    } else {
      SerialUSB.print("init(): failed, ret = ");
      SerialUSB.println(ret);
      delay(1000);
    }
  } while(true);

  sdp.startContinuous(true);
  sdp.startContinuousWait(true);
}

void loop() {
  int ret = sdp.readContinuous();
  if (ret == 0) {
    SerialUSB.print("Differential pressure: ");
    SerialUSB.print(sdp.getDifferentialPressure());
    SerialUSB.print("Pa | ");

    SerialUSB.print("Temp: ");
    SerialUSB.print(sdp.getTemperature());
    SerialUSB.print("C\n");
  } else {
    SerialUSB.print("Error in readSample(), ret = ");
    SerialUSB.println(ret);
  }

  delayMicroseconds(500);
}
