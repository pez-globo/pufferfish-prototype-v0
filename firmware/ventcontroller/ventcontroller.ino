//#include <TMCStepper.h>
//#include <TMCStepper_UTILITY.h>
#include <Wire.h>
// https://github.com/ethanjli/arduino-sdp
#include <sdpsensor.h>
#include <HoneywellTruStabilitySPI.h>

// HSCMRRN060MDSA3
//  - M       SMT
//  - RR      dual radial barbed ports, same side
//  - N       dry gas only, no diagnostics 
//  - 060MD   ±60 mbar
//  - S       SPI
//  - A       2e14 counts
//  - 3       3.3 Vcc

#define SLAVE_SELECT_PIN 0
TruStabilityPressureSensor pressure_sensor( SLAVE_SELECT_PIN, -61.183, 61.183 ); // unit: cmH2O
SDP8XXSensor sdp;

/*
// create Honeywell_ABP instance
Honeywell_ABP abp(
  0x28,   // I2C address
  0,      // minimum pressure
  100,      // maximum pressure
  "mbar"   // pressure unit
);
*/

static inline int sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

static const int CMD_LENGTH = 2;
static const int MSG_LENGTH = 6;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
static const int N_BYTES_POS = 3;

static const int pin_valve1 = 48;
static const int pin_valve2 = 49;

static const float flow_FS = 100;
static const float volume_FS = 1500;
static const float paw_FS = 50;
static const float Ti_FS = 5;
static const float Vt_FS = 1500;

static const float coefficient_dP2flow = 100;

float dP = 0;
float flow = 0;
float volume = 0;
float paw = 0;
float RR = 30;
float Ti = 0.5;
float Vt = 0;

float cycle_period_ms = 0; // duration of each breathing cycle
float cycle_time_ms = 0;  // current time in the breathing cycle
float time_inspiratory_ms = 500;
float frequency_send_data = 50;
float counter_send_data = 0;

volatile bool flag_send_data = false;
volatile bool flag_read_sensor = false;
uint16_t tmp_uint16;
int16_t tmp_int16;

#include <DueTimer.h>
static const float TIMER_PERIOD_us = 500; // in us

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established
  buffer_rx_ptr = 0;

  SPI.begin(); // start SPI communication
  pressure_sensor.begin(); // run sensor initialization
  
  Wire.begin();
  sdp.stopContinuous(); // stop continuous measurement if it's running
  // init sensirion sensor
  while(true)
  {
    int ret = sdp.init();
    if(ret == 0)
      break;
    else
      delay(100);
  }
  sdp.startContinuous(true);
  sdp.startContinuousWait(true);

  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  
  pinMode(pin_valve1, OUTPUT);
  digitalWrite(pin_valve1, LOW);

  pinMode(pin_valve2, OUTPUT);
  digitalWrite(pin_valve2, LOW);

  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

  cycle_period_ms = (60/RR)*1000;
  
}

void timer_interruptHandler()
{
  // read sensor value
  flag_read_sensor = true;
  
  // breathing control
  cycle_time_ms = cycle_time_ms + TIMER_PERIOD_us/1000;
  if(cycle_time_ms>cycle_period_ms)
  {
    cycle_time_ms = 0;
    // begin inspiratory flow
    volume = 0;
    set_valve2_state(0);
    set_valve1_state(1);
    digitalWrite(13,HIGH);
  }
  if(cycle_time_ms>time_inspiratory_ms)
  {
    // begin expiratory flow
    set_valve1_state(0);
    set_valve2_state(1);
    digitalWrite(13,LOW);
  }

  // send data to host computer
  counter_send_data = counter_send_data + 1;
  if((TIMER_PERIOD_us/1000000)*counter_send_data>=1/frequency_send_data)
  {
    counter_send_data = 0;
    flag_send_data = true;
  }
  
}

void loop() 
{
  while(SerialUSB.available()) 
  { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) 
    {
      buffer_rx_ptr = 0;
      if(buffer_rx[0]==0)
      {
        RR = buffer_rx[1];
        cycle_period_ms = (60/RR)*1000;
      }
      else if(buffer_rx[0]==1)
        Ti = (float(buffer_rx[1])/256)*Ti_FS;
      else if(buffer_rx[0]==2)
        Vt = (float(buffer_rx[1])/256)*Vt_FS;
      else if(buffer_rx[0]==3)
        set_valve1_state(buffer_rx[1]);
      else if(buffer_rx[0]==4)
        set_valve2_state(buffer_rx[1]);
    }
  }

  if(flag_read_sensor)
  {
    if(pressure_sensor.readSensor() == 0)
      paw = pressure_sensor.pressure();
    if(sdp.readContinuous() == 0)
      dP = sdp.getDifferentialPressure();
    flow = dP*coefficient_dP2flow;
    volume = volume + flow*1000*(TIMER_PERIOD_us/1000000/60);
    flag_read_sensor = false;
  }

  if(flag_send_data)
  {
    tmp_uint16 = 65536*paw/paw_FS;    
    buffer_tx[0] = byte(tmp_uint16>>8);
    buffer_tx[1] = byte(tmp_uint16%256);

    tmp_int16 = (65536/2)*flow/flow_FS;
    buffer_tx[2] = byte(tmp_int16>>8);
    buffer_tx[3] = byte(tmp_int16%256);

    tmp_uint16 = 65536*volume/volume_FS;
    buffer_tx[4] = byte(tmp_uint16>>8);
    buffer_tx[5] = byte(tmp_uint16%256);

    SerialUSB.write(buffer_tx,MSG_LENGTH);
    flag_send_data = false;

    /*  debug
    SerialUSB.print(paw);
    SerialUSB.print("\t ");
    SerialUSB.print(flow);
    SerialUSB.print("\t ");
    SerialUSB.println(volume);
    */
  }
}

void set_valve1_state(int state)
{
  if(state>0)
    digitalWrite(pin_valve1, HIGH);
  else
    digitalWrite(pin_valve1, LOW);
}

void set_valve2_state(int state)
{
  if(state>0)
    digitalWrite(pin_valve2, HIGH);
  else
    digitalWrite(pin_valve2, LOW);
}

// utils
long signed2NBytesUnsigned(long signedLong,int N)
{
  long NBytesUnsigned = signedLong + pow(256L,N)/2;
  return NBytesUnsigned;
}
