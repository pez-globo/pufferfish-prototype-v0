// SFM3x00 connection
//    Brown:    2 (RJ45)  SDA
//    Orange:   8 (RJ45)  GND
//    Blue:     4 (RJ45)  VCC (5V)
//    Green:    6 (RJ45)  SCK

#include <Wire.h>
#include <sfm3x00.h>
#include <HoneywellTruStabilitySPI.h>

// pin define
#define SLAVE_SELECT_PIN 0

SFM3000 sfm3000;
TruStabilityPressureSensor hsc_sensor( SLAVE_SELECT_PIN, 0, 351.5 ); // HSCDANN005PGSA3
float mFlow;
float mPressure;
int ret_sfm3000;
int ret_hsc_sensor;

void setup() 
{
  // wait for USB to connect
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established

  // set digital pins
  pinMode(1,OUTPUT);
  digitalWrite(1,HIGH);
  delayMicroseconds(500000);

  // initialize the SFM sensor
  Wire.begin();
  while(true) 
  {
    int ret = sfm3000.init();
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
  // get scale and offset factor for the SFM sensor
  sfm3000.get_scale_offset();

  // init the HSC sensor
  SPI.begin(); // start SPI communication
  hsc_sensor.begin(); // run sensor initialization
  
}

void loop() {
  ret_sfm3000 = sfm3000.read_sample();
  if (ret_sfm3000 == 0) 
    mFlow = sfm3000.get_flow();
  ret_hsc_sensor = hsc_sensor.readSensor();
  if (ret_hsc_sensor == 0)
    mPressure = hsc_sensor.pressure();

  if (ret_sfm3000 ==0 && ret_hsc_sensor == 0)
  {
    // SerialUSB.print("flow rate (slm): ");
    SerialUSB.print(mFlow);
    SerialUSB.print(",");
    //SerialUSB.print(" pressure (cmH2O): ");
    SerialUSB.print(mPressure);
    SerialUSB.print("\n");
  }
   
  delayMicroseconds(5000);
}
