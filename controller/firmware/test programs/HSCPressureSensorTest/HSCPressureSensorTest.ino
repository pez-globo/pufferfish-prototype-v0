/*
 * PressureSensorTest
 *
 * Fetch and print values from a Honeywell 
 * TruStability HSC Pressure Sensor over SPI
 * 
 * The sensor values used in this demo are 
 * for a -60 to 60 mbar gauge pressure sensor. 
 * 
 */

 // HSCMRRN060MDSA3
 // result: 5.5 s - 62,000 samples => > 11,000 samples/s

#include <HoneywellTruStabilitySPI.h>

#define SLAVE_SELECT_PIN 0
TruStabilityPressureSensor sensor( SLAVE_SELECT_PIN, -60.0, 60.0 );
float pressureReading;
int counter = 0;

void setup() {
  SerialUSB.begin(2000000);    
  while(!SerialUSB); 
  SerialUSB.println( "started" );
  SPI.begin(); // start SPI communication
  sensor.begin(); // run sensor initialization
}

void loop() {
  counter = 0;
  for(int i = 0; i < 100000; i++){
    // the sensor returns 0 when new data is ready
    if( sensor.readSensor() == 0 ) {
      //    SerialUSB.print( "temp [C]: " );
      //    SerialUSB.print( sensor.temperature() );
      //    SerialUSB.print( "\t pressure [psi]: " );
      //    SerialUSB.println( sensor.pressure() );
      pressureReading = sensor.pressure();
      counter = counter + 1;
    }
  }
  SerialUSB.print( "completed " );
  SerialUSB.print( counter );
  SerialUSB.print(" pressure measurements - " );
  SerialUSB.println( sensor.pressure() );
  // delay( 100 ); // Slow down sampling to 10 Hz. This is just a test.

}
