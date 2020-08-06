#include <Wire.h>
#include <sdpsensor-fast.h>
#include <sfm3x00.h>
#include "Honeywell_ABP.h"
#include "pwm_lib.h"
#include <DueTimer.h>
#include <sm9336.h>

/***************************************************************************************************/
/********************************************* Parameters ******************************************/
/***************************************************************************************************/
static const float TIMER_PERIOD_us = 1000; // in us
static const bool USE_SERIAL_MONITOR = false; // for debug
static const int MSG_LENGTH = 50*12;

static const int CMD_LENGTH = 4;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;

// full scale values
static const float flow_FS = 200;
static const float dP_FS = 500;

// measured variables
float mflow_sfm3000 = 0;
float mdP_sdp810 = 0;
float mdP_sdp31 = 0;
float mdP_sm9336 = 0;

volatile bool flag_log_data = false;
volatile bool flag_read_sensor = false;

// data logging
# define LOGGING_UNDERSAMPLING  1
volatile int counter_log_data = 0;

// other variables
uint16_t tmp_uint16;
int16_t tmp_int16;
long tmp_long;
volatile uint32_t timestamp = 0; // in number of TIMER_PERIOD_us

/***************************************************************************************************/
/********************************************* sensors *********************************************/
/***************************************************************************************************/
SFM3000 sfm3000;
SDP8XXSensor sdp810;
SDP3XSensor sdp31;
SM9336 sm9336;

int ret_sfm3000 = 0;
int ret_sdp810 = 0;
int ret_sdp31 = 0;

/***************************************************************************************************/
/******************************************* setup *************************************************/
/***************************************************************************************************/
void setup() 
{

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established
  buffer_rx_ptr = 0;
  buffer_tx_ptr = 0;

  Wire.setClock(350000);
  Wire.begin();

  // initialize the SFM sensor
  while(true) 
  {
    int ret_sfm3000 = sfm3000.init();
    if (ret_sfm3000 == 0) 
    {
      if(USE_SERIAL_MONITOR)
        SerialUSB.print("init() for sensor SFM3000 - 1: success\n");
      break;
    } 
    else 
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init() for sensor SFM3000 - 1: failed, ret = ");
        SerialUSB.println(ret_sfm3000);
      }
      delay(1000);
    }
  }
  sfm3000.get_scale_offset();
  sfm3000.start_continuous();

  // initialize the SDP810 sensor
  sdp810.stopContinuous(); // stop continuous measurement if it's running
  while (true)
  {
    int ret = sdp810.init();
    if (ret == 0)
    {
      if(USE_SERIAL_MONITOR)
        SerialUSB.print("init() for SDP810: success\n");
      break;
    }
    else
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init() for SDP810: failed, ret = ");
        SerialUSB.println(ret);
      }
      delay(1000);
    }
  }
  sdp810.startContinuous(true);
  sdp810.startContinuousWait(true);

  // initialize the SDP31 sensor
  sdp31.stopContinuous(); // stop continuous measurement if it's running
  while (true)
  {
    int ret = sdp31.init();
    if (ret == 0)
    {
      if(USE_SERIAL_MONITOR)
        SerialUSB.print("init() for SDP31: success\n");
      break;
    }
    else
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init() for SDP31: failed, ret = ");
        SerialUSB.println(ret);
      }
      delay(1000);
    }
  }
  sdp31.startContinuous(true);
  sdp31.startContinuousWait(true);

  delayMicroseconds(500000);

  // start the timer
  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

}

/***************************************************************************************************/
/******************************** timer interrupt handling routine *********************************/
/***************************************************************************************************/
void timer_interruptHandler()
{
  timestamp = timestamp + 1;

  // read sensor value
  flag_read_sensor = true;
  
  // send data to host computer
  counter_log_data = counter_log_data + 1;
  if (counter_log_data >= LOGGING_UNDERSAMPLING)
  {
    counter_log_data = 0;
    flag_log_data = true;
  }
}

/***************************************************************************************************/
/********************************************  main loop *******************************************/
/***************************************************************************************************/
void loop()
{
  if (flag_read_sensor)
  {
    int ret_sfm3000 = sfm3000.read_sample();
    if (ret_sfm3000 == 0)
      mflow_sfm3000 = sfm3000.get_flow();

    if (sdp31.readContinuous() == 0)
      mdP_sdp31 = sdp31.getDifferentialPressure();
    else
      mdP_sdp31 = 0;

    if (sdp810.readContinuous() == 0)
      mdP_sdp810 = sdp810.getDifferentialPressure();
    else
      mdP_sdp810 = 0;

    if(sm9336.read_sample() == 0)
      mdP_sm9336 = sm9336.get_dP();
    else
      mdP_sm9336 = 0;
    
    flag_read_sensor = false;
  }

  if (flag_log_data)
  {
    flag_log_data = false;

    // field 1: time
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 24);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 16);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp %256);

    // field 2 flow - SFM3000
    tmp_long = (65536 / 2) * mflow_sfm3000 / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 3 dP: SDP31
    tmp_long = (65536 / 2) * mdP_sdp31 / dP_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 4 dP - SDP810
    tmp_long = (65536 / 2) * mdP_sdp810 / dP_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 5 dP - SM9336
    tmp_long = (65536 / 2) * mdP_sm9336 / dP_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);
    
    if (buffer_tx_ptr == MSG_LENGTH)
    {
      buffer_tx_ptr = 0;
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print(mflow_sfm3000);
        SerialUSB.print('\t');
        SerialUSB.print(mdP_sdp31);
        SerialUSB.print('\t');
        SerialUSB.print(mdP_sdp810);
        SerialUSB.print('\t');
        SerialUSB.println(mdP_sm9336);
      }
      else
        SerialUSB.write(buffer_tx, MSG_LENGTH);
    }
  }
}

/***************************************************************************************************/
/*********************************************  utils  *********************************************/
/***************************************************************************************************/
static inline int sgn(int val) 
{
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

long signed2NBytesUnsigned(long signedLong, int N)
{
  long NBytesUnsigned = signedLong + pow(256L, N) / 2;
  return NBytesUnsigned;
}
