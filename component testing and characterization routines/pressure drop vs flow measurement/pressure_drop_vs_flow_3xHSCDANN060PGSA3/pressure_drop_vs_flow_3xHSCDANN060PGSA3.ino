// SFM3x00 connection
//    Brown:    2 (RJ45)  SDA
//    Orange:   8 (RJ45)  GND
//    Blue:     4 (RJ45)  VCC (5V)
//    Green:    6 (RJ45)  SCK

#include <Wire.h>
#include <sfm3x00.h>
#include <HoneywellTruStabilitySPI.h>

// pin define
#define SLAVE_SELECT_PIN_1 30
#define SLAVE_SELECT_PIN_2 31
#define SLAVE_SELECT_PIN_3 32
#define SLAVE_SELECT_PIN_4 33

SFM3000 sfm3000;

//HSCDANN060PGSA3
TruStabilityPressureSensor hsc_sensor_1( SLAVE_SELECT_PIN_1, 0, 60 ); // HSCDANN005PGSA3
TruStabilityPressureSensor hsc_sensor_2( SLAVE_SELECT_PIN_2, 0, 60 ); // HSCDANN005PGSA3
TruStabilityPressureSensor hsc_sensor_3( SLAVE_SELECT_PIN_3, 0, 60 ); // HSCDANN005PGSA3
TruStabilityPressureSensor hsc_sensor_4( SLAVE_SELECT_PIN_4, 0, 60 ); // HSCDANN005PGSA3

float mFlow;
float mPressure_1;
float mPressure_2;
float mPressure_3;

int ret_sfm3000;
int ret_hsc_sensor_1;
int ret_hsc_sensor_2;
int ret_hsc_sensor_3;

void setup() 
{
  // wait for USB to connect
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established

//  // initialize the SFM sensor
//  Wire.begin();
//  while(true) 
//  {
//    int ret = sfm3000.init();
//    if (ret == 0) {
//      SerialUSB.print("init(): success\n");
//      break;
//    } 
//    else 
//    {
//      SerialUSB.print("init(): failed, ret = ");
//      SerialUSB.println(ret);
//      delay(1000);
//    }
//  }
//  // get scale and offset factor for the SFM sensor
//  sfm3000.get_scale_offset();

  // init the HSC sensor
  SPI.begin(); // start SPI communication
  hsc_sensor_1.begin(); // run sensor initialization
  hsc_sensor_2.begin(); // run sensor initialization
  hsc_sensor_3.begin(); // run sensor initialization
  
}

void loop() 
{
  
//  ret_sfm3000 = sfm3000.read_sample();
  ret_hsc_sensor_1 = hsc_sensor_1.readSensor();
  ret_hsc_sensor_2 = hsc_sensor_2.readSensor();
  ret_hsc_sensor_3 = hsc_sensor_3.readSensor();
  // ret_hsc_sensor_4 = hsc_sensor_1.readSensor();
  
//  if (ret_sfm3000 == 0) 
//    mFlow = sfm3000.get_flow();

  ret_sfm3000 = 0;
  
  if (ret_hsc_sensor_1+ret_hsc_sensor_2+ret_hsc_sensor_3 == 0)
  {
    mPressure_1 = hsc_sensor_1.pressure();
    mPressure_2 = hsc_sensor_2.pressure();
    mPressure_3 = hsc_sensor_3.pressure();
  }

  if (ret_sfm3000 + ret_hsc_sensor_1 + ret_hsc_sensor_2 + ret_hsc_sensor_3 == 0)
  {
    // SerialUSB.print("flow rate (slm): ");
    SerialUSB.print(mFlow);
    SerialUSB.print(",");
    //SerialUSB.print(" pressure (cmH2O): ");
    SerialUSB.print(mPressure_1);
    SerialUSB.print(",");
    SerialUSB.print(mPressure_2);
    SerialUSB.print(",");
    SerialUSB.print(mPressure_3);
    SerialUSB.print("\n");
  }
   
  delayMicroseconds(5000);
}
