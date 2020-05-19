#include <Wire.h>
#include <sfm3x00.h>
//#include "sfm3x00.h"

SFM3200 sfm3200;

void setup() {
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established

  pinMode(1,OUTPUT);
  digitalWrite(1,HIGH);
  delayMicroseconds(500000);
  
  Wire.begin();
  // initialize the sensor
  while(true) 
  {
    int ret = sfm3200.init();
    if (ret == 0) {
      SerialUSB.print("init(): success\n");
      break;
    } 
    else 
    {
      SerialUSB.print("init(): failed, ret = ");
      SerialUSB.println(ret);
      delay(1000);
    }
  }
  // get scale and offset factor
  sfm3200.get_scale_offset();
}

void loop() {
  int ret = sfm3200.read_sample();
  if (ret == 0) 
  {
    SerialUSB.print("flow rate: ");
    SerialUSB.print(sfm3200.get_flow());
    SerialUSB.print("slm \n");
  } 
  //  else {
  //    SerialUSB.print("Error in readSample(), ret = ");
  //    SerialUSB.println(ret);
  //  }
  delayMicroseconds(5000);
}
