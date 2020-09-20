#include <Wire.h>
#include <sfm3x00.h>
#include <DueTimer.h>
#include "pwm_lib.h"
#include "Honeywell_ABP.h"

static const bool USE_SERIAL_MONITOR = false;

static const float TIMER_PERIOD_us = 500; // in us
static const int timer_div = 1;
volatile int timer_div_counter = 0;
volatile uint32_t cycle_count = 0;
volatile uint32_t timestamp = 0; // in number of TIMER_PERIOD_us

volatile bool flag_read_sensor = false;
volatile bool flag_write_data = false;
volatile bool flag_update_pwm = false;
volatile bool is_cycling = false;

/*******************************************************************
 *************************** Communication *************************
 *******************************************************************/
static const int CMD_LENGTH = 16;
static const int MSG_LENGTH = 15*32;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

long tmp_long;
uint16_t tmp_uint16;
float flow_FS = 200;
float pressure_FS = 60;

/***************************************************************************************************/
/********************************************* pwm_lib *********************************************/
/***************************************************************************************************/
// https://github.com/antodom/pwm_lib
// http://www.robgray.com/temp/Due-pinout.pdf
using namespace arduino_due::pwm_lib;

#define PWM_FREQUENCY 10000
#define PWM_PERIOD 10000 // 10000 x (1e-8 secs) = 1e-4 sec
#define PWM_DUTY 0

// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWMH0_PC3> pwm_pin35;
pwm_wrapper<decltype(pwm_pin35)> pwm_wrapper_pin35(pwm_pin35);
pwm_base* pwm_wrapper_pin35_ptr=&pwm_wrapper_pin35;

/*******************************************************************
 ***************************** SENSORS *****************************
 *******************************************************************/
SFM3000 sfm3000;
Honeywell_ABP abp_60psi_1(
  0x28,   // I2C address
  0,      // minimum pressure
  60,      // maximum pressure
  "psi"   // pressure unit
);
int ret_sfm3000 = 0;
float mFlow;
float mPressure;

/*******************************************************************
 **************************** variables ****************************
 *******************************************************************/
float flow_start = 0;
float flow_end = 0;
float flow_step_increase = 0;
float flow_step_decrease = 0;
float flow_max_hold = 0;
float flow_min_hold = 0;
uint32_t flow_cycle_N = 0;
uint32_t flow_cycle_current = 0;

int phase = 0;
int counter_phase = 0;

float valve_opening = 0;
float P_gain = 0.00001;
float I_gain = 0.0005;
float flow_setpoint = 0;
float error = 0;
float error_integral = 0;

/*******************************************************************
 ******************************* SETUP *****************************
 *******************************************************************/
void setup() 
{

  pwm_wrapper_pin35_ptr->start(PWM_PERIOD,PWM_DUTY);
  
  // wait for USB to connect
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established

  // reset rx buffer pointer
  buffer_rx_ptr = 0;

  // init stepper motors
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);

  // initialize the SFM sensor
  Wire.begin();
  while(true) 
  {
    int ret = sfm3000.init();
    if (ret == 0) 
    {
      if(USE_SERIAL_MONITOR)
        SerialUSB.print("init(): success\n");
      break;
    } 
    else 
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init(): failed, ret = ");
        SerialUSB.println(ret);
      }
      delay(1000);
    }
  }
  // get scale and offset factor for the SFM sensor
  sfm3000.get_scale_offset();
  sfm3000.start_continuous();

  delayMicroseconds(500000);

  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);
  
}

void loop() 
{
  // read one meesage from the buffer
  while (SerialUSB.available()) 
  { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) 
    {
      buffer_rx_ptr = 0;
      flow_start = ((float(buffer_rx[0]*256)+float(buffer_rx[1]))/65535)*flow_FS;
      flow_end = ((float(buffer_rx[2]*256)+float(buffer_rx[3]))/65535)*flow_FS;
      flow_step_increase = ((float(buffer_rx[4]*256)+float(buffer_rx[5]))/65535)*flow_FS;
      flow_step_decrease = ((float(buffer_rx[6]*256)+float(buffer_rx[7]))/65535)*flow_FS;
      flow_max_hold = uint16_t(buffer_rx[8]*256)+uint16_t(buffer_rx[9]);
      flow_min_hold = uint16_t(buffer_rx[10]*256)+uint16_t(buffer_rx[11]);
      flow_cycle_N = uint32_t(buffer_rx[12])*256*256*256+uint32_t(buffer_rx[13])*256*256+uint32_t(buffer_rx[14])*256+uint32_t(buffer_rx[15]);

      if(flow_cycle_N>0)
      {
        is_cycling = true;
        flow_cycle_current = 0;
        phase = 0;
        flow_setpoint = flow_start;
      }
      else
        flow_setpoint = flow_start;      
    }
  }

  // read sensor
  if(flag_read_sensor)
  {
    ret_sfm3000 = sfm3000.read_sample();
    if (ret_sfm3000 == 0) 
    {
      mFlow = sfm3000.get_flow();
      // low pass filter 
    }
    abp_60psi_1.update();
    mPressure = abp_60psi_1.pressure();
    flag_read_sensor = false;

    // PID control of flow
    error = flow_setpoint - mFlow;
    error_integral = error_integral + error;
    error_integral > 200*I_gain ? 200*I_gain : error_integral;
    error_integral < 0 ? 0 : error_integral;
    
    valve_opening = error*P_gain + error_integral*I_gain;
    valve_opening = valve_opening > 1 ? 1 : valve_opening;
    valve_opening = valve_opening < 0 ? 0 : valve_opening;
    set_valve_opening(valve_opening);
  }

  // write data
  if(flag_write_data)
  {
    // only write data if the last read is successful
    if (ret_sfm3000 == 0)
    {
      if(USE_SERIAL_MONITOR)
      {
        // SerialUSB.print("flow rate (slm): ");
        SerialUSB.print(mFlow);
        SerialUSB.print(",");
        //SerialUSB.print(" pressure (cmH2O): ");
        SerialUSB.println(mPressure);
      }
      else
      {
        // field 1: time
        buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 24);
        buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 16);
        buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(timestamp %256);

        // field 2: cycle
        buffer_tx[buffer_tx_ptr++] = byte(flow_cycle_current >> 24);
        buffer_tx[buffer_tx_ptr++] = byte(flow_cycle_current >> 16);
        buffer_tx[buffer_tx_ptr++] = byte(flow_cycle_current >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(flow_cycle_current %256);

        // field 3: PWM
        tmp_uint16 = flow_setpoint;
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

        // field 4: flow
        tmp_long = (65536 / 2) * mFlow / flow_FS;
        tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

        // field 5: upstream pressure
        tmp_uint16 = 65536 * mPressure / pressure_FS;
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

        // filed 6: cycling
        buffer_tx[buffer_tx_ptr++] = is_cycling;
      }
    }
    
    // send data to computer
    if(buffer_tx_ptr==MSG_LENGTH)
    {
      SerialUSB.write(buffer_tx, MSG_LENGTH);
      buffer_tx_ptr = 0;
    }
    if(buffer_tx_ptr>MSG_LENGTH)
      buffer_tx_ptr = 0;
    
    // clear the flag
    flag_write_data = false;
  }

  if(flag_update_pwm)
  {
    // cycling control
    if(is_cycling)
    {
      if(phase == 0)
      {
        flow_setpoint = flow_setpoint + flow_step_increase;
        if(flow_setpoint >= flow_end)
        {
          flow_setpoint = flow_end;
          phase = 1;
          counter_phase = 0;
        }
      }
      if(phase == 1)
      {
        counter_phase = counter_phase + 1;
        if(counter_phase>flow_max_hold)
          phase = 2;
      }
      if(phase == 2)
      {
        flow_setpoint = flow_setpoint - flow_step_decrease;
        if(flow_setpoint <= flow_start)
        {
          flow_setpoint = flow_start;
          phase = 3;
          counter_phase = 0;
        }
      }
      if(phase == 3)
      {
        counter_phase = counter_phase + 1;
        if(counter_phase>flow_min_hold)
        {
          flow_cycle_current = flow_cycle_current + 1;
          if(flow_cycle_current >= flow_cycle_N)
            is_cycling = false;
          else
            phase = 0;
        }
      }
    }
    flag_update_pwm = false;
  }
}

void timer_interruptHandler()
{
  
  timestamp = timestamp + 1;
  flag_read_sensor = true;
  flag_write_data = true;
  flag_update_pwm = true;
  
}

/***************************************************************************************************/
/********************************************* valves **********************************************/
/***************************************************************************************************/
void set_valve_opening(float opening)
{
  if(opening>1)
    opening = 1;
  change_duty(*pwm_wrapper_pin35_ptr,opening*PWM_PERIOD,PWM_PERIOD);
}

/***************************************************************************************************/
/*********************************************  utils  *********************************************/
/***************************************************************************************************/long signed2NBytesUnsigned(long signedLong,int N)
{
  long NBytesUnsigned = signedLong + pow(256L,N)/2;
  //long NBytesUnsigned = signedLong + 8388608L;
  return NBytesUnsigned;
}

static inline int sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

/***************************************************************************************************/
/********************************************* pwm_lib *********************************************/
/***************************************************************************************************/
void change_duty(pwm_base& pwm_obj, uint32_t pwm_duty, uint32_t pwm_period) 
{ 
  uint32_t duty = pwm_duty;
  if(duty>pwm_period) 
    duty=pwm_duty; 
  pwm_obj.set_duty(duty); 
}
