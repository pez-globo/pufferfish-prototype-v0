#include <Wire.h>
#include <sdpsensor-fast.h>
#include <sfm3x00.h>
#include "Honeywell_ABP.h"
#include "pwm_lib.h"
#include <DueTimer.h>

#define MUX_ADDR 0x70 //7-bit unshifted default I2C Address

/***************************************************************************************************/
/********************************************* Parameters ******************************************/
/***************************************************************************************************/
static const float TIMER_PERIOD_us = 1000; // in us
static const bool USE_SERIAL_MONITOR = false; // for debug
static const int MSG_LENGTH = 50*8;

static const int CMD_LENGTH = 4;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;

// full scale values
static const float flow_FS = 200;
static const float paw_FS = 100;

// measured variables
float mflow_1 = 0;
float mpaw = 0;

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
SFM3000 sfm3200_1;
SDP8XXSensor sdp;

// create Honeywell_ABP instance
Honeywell_ABP abp_1psi_1(
  0x28,   // I2C address
  0,      // minimum pressure
  1,      // maximum pressure
  "psi"   // pressure unit
);

uint8_t cmd[1];
int ret_sfm3200_1 = 0;

/***************************************************************************************************/
/******************************************* setup *************************************************/
/***************************************************************************************************/
void setup() 
{

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established
  buffer_rx_ptr = 0;

  Wire.setClock(400000);
  Wire.begin();

  // initialize the SFM sensor 1
  while(true) 
  {
    int ret_sfm3200_1 = sfm3200_1.init();
    if (ret_sfm3200_1 == 0) 
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
        SerialUSB.println(ret_sfm3200_1);
      }
      delay(1000);
    }
  }
  sfm3200_1.get_scale_offset();
  sfm3200_1.start_continuous();

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
    int ret_sfm3200_1 = sfm3200_1.read_sample();
    if (ret_sfm3200_1 == 0)
      mflow_1 = sfm3200_1.get_flow();

    abp_1psi_1.update();
    mpaw = abp_1psi_1.pressure()*70.307;
    
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

    // field 2 flow - 1
    tmp_long = (65536 / 2) * mflow_1 / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 3 airway pressure
    tmp_long = (65536 / 2) * mpaw / paw_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    if (buffer_tx_ptr == MSG_LENGTH)
    {
      buffer_tx_ptr = 0;
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.println(mflow_1);
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
