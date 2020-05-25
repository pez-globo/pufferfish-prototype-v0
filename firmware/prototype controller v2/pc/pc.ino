#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <Wire.h>
//#include <sfm3x00.h>
// https://github.com/ethanjli/arduino-sdp
#include <sdpsensor.h>
#include <HoneywellTruStabilitySPI.h>
#include <AccelStepper.h>

static const bool USE_SERIAL_MONITOR = false;

// HSCMRRN060MDSA3
//  - M       SMT
//  - RR      dual radial barbed ports, same side
//  - N       dry gas only, no diagnostics
//  - 060MD   ±60 mbar
//  - S       SPI
//  - A       2e14 counts
//  - 3       3.3 Vcc

#define SLAVE_SELECT_PIN 1
TruStabilityPressureSensor pressure_sensor( SLAVE_SELECT_PIN, -61.183, 61.183 ); // unit: cmH2O
//SFM3000 sfm3000;
SDP8XXSensor sdp;

#include <DueTimer.h>
static const float TIMER_PERIOD_us = 1000; // in us

static inline int sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

static const int CMD_LENGTH = 3;
static const int MSG_LENGTH = 8;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
static const int N_BYTES_POS = 3;

// pin def
static const int pin_valve2 = 11;

// command sets
static const uint8_t CMD_Vt = 0;
static const uint8_t CMD_Ti = 1;
static const uint8_t CMD_RR = 2;
static const uint8_t CMD_PEEP = 3;
static const uint8_t CMD_Flow = 4;
static const uint8_t CMD_Pinsp = 5;
static const uint8_t CMD_RiseTime = 6;
static const uint8_t CMD_PID_P = 7;
static const uint8_t CMD_PID_I_frac = 8;
    
static const float flow_FS = 200;
static const float volume_FS = 1500;
static const float paw_FS = 50;
static const float Ti_FS = 5;
static const float Vt_FS = 1500;
static const float PEEP_FS = 30;
static const float RR_FS = 60;
static const float valve_pos_open_steps_FS = -125;

static const float pc_rise_time_ms_FS = 500;
static const float PID_coefficient_P_FS = 0.1;
static const float PID_coefficient_I_frac_FS = 1;

float valve_pos_open_steps = -100;

static const float coefficient_dP2flow = 0.6438;
static const float coefficient_dp2flow_offset = 1.1029;

volatile float dP = 0;
volatile float mflow = 0;
volatile float mvolume = 0;
volatile float mpaw = 0;

float RR = 24;
float Ti = 1.2;
float Vt = 250;
float PEEP = 5;
float paw_trigger_th = 3;
float pc_rise_time_ms = 100;
float paw_setpoint = 20;

float PID_coefficient_P = 0.01;
float PID_coefficient_I = 0.001;
float PID_coefficient_I_frac = 0.1;
float PID_coefficient_D = 0;
volatile float paw_setpoint_rise = 0;

volatile float mPEEP = 0;
volatile float PID_Insp_Integral = 0;
volatile float PID_Insp_Prop = 0;

float cycle_period_ms = 0; // duration of each breathing cycle
float cycle_time_ms = 0;  // current time in the breathing cycle
float time_inspiratory_ms = Ti * 1000;
float frequency_send_data = 50;
float counter_send_data = 0;

volatile bool flag_send_data = false;
volatile bool flag_read_sensor = false;

volatile bool is_breathing = true;
volatile bool is_in_inspiratory_phase = false;
volatile bool is_in_pressure_regulation = false;
volatile bool is_in_expiratory_phase = false;
volatile bool is_in_pressure_regulation_rise = false;
volatile bool is_in_pressure_regulation_plateau = false;
volatile bool PEEP_is_reached = false;


// pressure control variables
volatile float paw_error = 0;
volatile float paw_error_integrated = 0;
volatile float paw_error_differential = 0;

// valve control
volatile float valve_opening = 0; // 0 - 1

uint16_t tmp_uint16;
int16_t tmp_int16;
long tmp_long;

uint16_t timebase = 0; // in number of TIMER_PERIOD_us
static const long DISPLAY_RANGE_S = 20;

// stepper
static const int UART_CS_S0 = 46;
static const int UART_CS_S1 = 47;
#define STEPPER_SERIAL Serial3
static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;
TMC2209Stepper Z_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
static const int Z_dir = 27;
static const int Z_step = 29;
static const int Z_N_microstepping = 2;
static const long steps_per_mm_Z = 30*Z_N_microstepping; 
constexpr float MAX_VELOCITY_Z_mm = 25; 
constexpr float MAX_ACCELERATION_Z_mm = 2000; // 12.5 ms to reach max speed
AccelStepper stepper_Z = AccelStepper(AccelStepper::DRIVER, Z_step, Z_dir);

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established
  buffer_rx_ptr = 0;

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(pin_valve2, OUTPUT);
  analogWrite(pin_valve2, 0);

  // initialize the Honeywell pressure sensor
  SPI.begin(); // start SPI communication
  pressure_sensor.begin(); // run sensor initialization

  // initialize the SDP sensor
  Wire.begin();
  sdp.stopContinuous(); // stop continuous measurement if it's running
  while (true)
  {
    int ret = sdp.init();
    if (ret == 0)
      break;
    else
      delay(100);
  }
  sdp.startContinuous(true);
  sdp.startContinuousWait(true);

  /*
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
  */

  // stepper driver init.
  pinMode(Z_dir, OUTPUT);
  pinMode(Z_step, OUTPUT);
  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);
  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);
  select_driver(4);
  while(!STEPPER_SERIAL);
  Z_driver.begin();
  Z_driver.I_scale_analog(false);  
  Z_driver.rms_current(300,0.2); //I_run and holdMultiplier
  Z_driver.microsteps(Z_N_microstepping);
  Z_driver.pwm_autoscale(true);
  Z_driver.TPOWERDOWN(2);
  Z_driver.en_spreadCycle(false);
  Z_driver.toff(4);
  stepper_Z.setPinsInverted(false, false, true);
  stepper_Z.setMaxSpeed(MAX_VELOCITY_Z_mm*steps_per_mm_Z);
  stepper_Z.setAcceleration(MAX_ACCELERATION_Z_mm*steps_per_mm_Z);
  stepper_Z.enableOutputs();

  // calculate the cycle period
  cycle_period_ms = (60 / RR) * 1000;

  // start the timer
  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

}

void timer_interruptHandler()
{
  timebase = timebase + 1;
  if (timebase >= (DISPLAY_RANGE_S * 1000000 / TIMER_PERIOD_us))
    timebase = 0;

  // read sensor value
  flag_read_sensor = true;

  // update cycle timer
  cycle_time_ms = cycle_time_ms + TIMER_PERIOD_us / 1000;

  if (is_breathing)
  {
    // time-triggered breath
    if (cycle_time_ms > cycle_period_ms && is_in_inspiratory_phase == false )
    {
      cycle_time_ms = 0;
      is_in_inspiratory_phase = true;
      is_in_pressure_regulation_rise = true;
      is_in_expiratory_phase = false;
      PEEP_is_reached = false;
      mvolume = 0;
      mPEEP = mpaw;
      set_valve2_state(0);
      digitalWrite(13, HIGH);
      PID_Insp_Integral = 0;
    }

    /*
    // patient triggered breath
    if ( mpaw < paw_trigger_th && is_in_expiratory_phase && is_in_inspiratory_phase == false )
    {
      cycle_time_ms = 0;
      is_in_inspiratory_phase = true;
      is_in_expiratory_phase = false;
      PEEP_is_reached = false;
      mvolume = 0;
      set_valve2_state(0);
      set_valve1_state(1);
      digitalWrite(13, HIGH);
    }
    */

    /*
    // generate decelerating waveform
    if (cycle_time_ms > 200 && is_in_inspiratory_phase )
    {
      valve_opening_percentage = valve_opening_percentage - decelerating_rate;
      stepper_Z.moveTo(-valve_opening_percentage*travel*steps_per_mm_XY);
    }

    // breathing control - stop inspiratory mflow when Vt is reached
    if (mvolume >= Vt && is_in_inspiratory_phase == true)
    {
      set_valve1_state(0);
      is_in_inspiratory_phase = false;
    }
    */

    if(is_in_pressure_regulation_rise == true && cycle_time_ms <= time_inspiratory_ms)
    {
      // determine state
      if(cycle_time_ms < pc_rise_time_ms + 200 && mpaw < 0.95*paw_setpoint && is_in_pressure_regulation_plateau == false)
        is_in_pressure_regulation_rise = true;
      else
      {
        is_in_pressure_regulation_rise = false;
        is_in_pressure_regulation_plateau = true;
      }
      
      // calculate setpoint
      paw_setpoint_rise = (paw_setpoint-mPEEP)*(cycle_time_ms/pc_rise_time_ms) + mPEEP;
      // calculate error
      paw_error = paw_setpoint_rise - mpaw;
      PID_Insp_Integral = PID_Insp_Integral + 0.7*PID_coefficient_I*paw_error;
      PID_Insp_Integral = PID_Insp_Integral > 1 ? 1 : PID_Insp_Integral;
      PID_Insp_Integral = PID_Insp_Integral < 0 ? 0 : PID_Insp_Integral;
      PID_Insp_Prop = 0.7*PID_coefficient_P*paw_error;
      // generate command
      valve_opening = PID_Insp_Prop + PID_Insp_Integral;
      valve_opening = valve_opening > 1 ? 1 : valve_opening;
      valve_opening = valve_opening < 0 ? 0 : valve_opening;
      set_valve1_pos(valve_opening);
    }

    if(is_in_pressure_regulation_plateau && cycle_time_ms <= time_inspiratory_ms)
    {
      // calculate error
      paw_error = paw_setpoint - mpaw;
      PID_Insp_Integral = PID_Insp_Integral + PID_coefficient_I*paw_error;
      PID_Insp_Integral = PID_Insp_Integral > 1 ? 1 : PID_Insp_Integral;
      PID_Insp_Integral = PID_Insp_Integral < 0 ? 0 : PID_Insp_Integral;
      PID_Insp_Prop = PID_coefficient_P*paw_error;
      // generate command
      valve_opening = PID_Insp_Prop + PID_Insp_Integral;
      valve_opening = valve_opening > 1 ? 1 : valve_opening;
      valve_opening = valve_opening < 0 ? 0 : valve_opening;
      set_valve1_pos(valve_opening);
    }

    // breathing control - change to exhalation when Ti is reached
    if (cycle_time_ms > time_inspiratory_ms && is_in_expiratory_phase == false)
    {
      digitalWrite(13, LOW);
      is_in_inspiratory_phase = false;
      is_in_pressure_regulation_rise = false;
      is_in_pressure_regulation_plateau = false;
      is_in_expiratory_phase = true;
      valve_opening = 0;
      set_valve1_pos(valve_opening);
    }

    if (is_in_expiratory_phase == true)
    { 
      /*
      if (mpaw > PEEP && PEEP_is_reached == false)
        set_valve2_state(1);
      else
      {
        set_valve2_state(0);
        PEEP_is_reached = true;
      }
      */
      set_valve2_state(1);
    }
  }

  // send data to host computer
  counter_send_data = counter_send_data + 1;
  if ((TIMER_PERIOD_us / 1000000)*counter_send_data >= 1 / frequency_send_data)
  {
    counter_send_data = 0;
    flag_send_data = true;
  }

}

void loop()
{
  while (SerialUSB.available())
  {
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH)
    {
      buffer_rx_ptr = 0;
      if (buffer_rx[0] == CMD_RR)
      {
        RR = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*RR_FS;
        cycle_period_ms = (60 / RR) * 1000;
      }
      else if (buffer_rx[0] == CMD_Ti)
      {
        Ti = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*Ti_FS;
        time_inspiratory_ms = Ti * 1000;
      }
      else if (buffer_rx[0] == CMD_Vt)
        Vt = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*Vt_FS;
      else if (buffer_rx[0] == CMD_PEEP)
        PEEP = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*PEEP_FS;
      else if (buffer_rx[0] == CMD_Flow)
        valve_pos_open_steps = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * valve_pos_open_steps_FS;
      else if (buffer_rx[0] == CMD_Pinsp)
        paw_setpoint =  ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * paw_FS;
      else if (buffer_rx[0] == CMD_RiseTime)
        pc_rise_time_ms =  ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * pc_rise_time_ms_FS;
      else if (buffer_rx[0] == CMD_PID_P)
      {
        PID_coefficient_P = PID_coefficient_P_FS*(float(buffer_rx[1])*256+float(buffer_rx[2]))/65536.0;
        PID_coefficient_I = PID_coefficient_P*PID_coefficient_I_frac;
      }
      else if (buffer_rx[0] == CMD_PID_I_frac)
      {
        PID_coefficient_I_frac = PID_coefficient_I_frac_FS*(float(buffer_rx[1])*256+float(buffer_rx[2]))/65536.0;
        PID_coefficient_I = PID_coefficient_P*PID_coefficient_I_frac;
      }
    }
  }

  if (flag_read_sensor)
  {
    if (pressure_sensor.readSensor() == 0)
      mpaw = pressure_sensor.pressure();
    if (sdp.readContinuous() == 0)
      dP = sdp.getDifferentialPressure();
    if(abs(dP)<0.3)
      mflow = 0;
    else if(dP >=3 )
      mflow = dP * coefficient_dP2flow + coefficient_dp2flow_offset;
    else
      mflow = dP * coefficient_dP2flow - coefficient_dp2flow_offset;
    mvolume = mvolume + mflow * 1000 * (TIMER_PERIOD_us / 1000000 / 60);
    flag_read_sensor = false;
  }

  if (flag_send_data)
  {
    tmp_long = (65536 / 2) * mpaw / paw_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[0] = byte(tmp_uint16 >> 8);
    buffer_tx[1] = byte(tmp_uint16 % 256);

    tmp_long = (65536 / 2) * mflow / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[2] = byte(tmp_uint16 >> 8);
    buffer_tx[3] = byte(tmp_uint16 % 256);

    tmp_uint16 = 65536 * mvolume / volume_FS;
    buffer_tx[4] = byte(tmp_uint16 >> 8);
    buffer_tx[5] = byte(tmp_uint16 % 256);

    buffer_tx[6] = byte(timebase >> 8);
    buffer_tx[7] = byte(timebase % 256);

    SerialUSB.write(buffer_tx, MSG_LENGTH);
    flag_send_data = false;

    //    SerialUSB.print(mpaw);
    //    SerialUSB.print("\t ");
    //    SerialUSB.print(mflow);
    //    SerialUSB.print("\t ");
    //    SerialUSB.println(mvolume);
  }
  
  stepper_Z.run();
  
}

void set_valve1_pos(float pos)
{
  stepper_Z.moveTo(valve_pos_open_steps * pos);
}

float get_valve1_pos()
{
  return stepper_Z.currentPosition();
}

void set_valve2_state(int state)
{
  if (state > 0)
    analogWrite(pin_valve2, 125);
  else
    analogWrite(pin_valve2, 255);
}

// utils
long signed2NBytesUnsigned(long signedLong, int N)
{
  long NBytesUnsigned = signedLong + pow(256L, N) / 2;
  return NBytesUnsigned;
}

void select_driver(int id)
{
  if(id==1)
  {
    digitalWrite(UART_CS_S0, LOW);
    digitalWrite(UART_CS_S1, LOW);
  }
  if(id==2)
  {
    digitalWrite(UART_CS_S0, HIGH);
    digitalWrite(UART_CS_S1, LOW);
  }
  if(id==3)
  {
    digitalWrite(UART_CS_S0, LOW);
    digitalWrite(UART_CS_S1, HIGH);
  }
  if(id==4)
  {
    digitalWrite(UART_CS_S0, HIGH);
    digitalWrite(UART_CS_S1, HIGH);
  }
}
