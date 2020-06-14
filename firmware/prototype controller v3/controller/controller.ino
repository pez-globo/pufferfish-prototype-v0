#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <Wire.h>
#include <sdpsensor-fast.h>
#include <sfm3x00.h>
#include <AccelStepper.h>
#include "Honeywell_ABP.h"
#include "pwm_lib.h"
#include <DueTimer.h>

#define MUX_ADDR 0x70 //7-bit unshifted default I2C Address

/***************************************************************************************************/
/********************************************* Parameters ******************************************/
/***************************************************************************************************/
static const uint32_t EXHALATION_CONTROL_DUTY_CLOSE = 9000;
uint32_t exhalation_control_PEEP_duty = 5000;
static const float TIMER_PERIOD_us = 1400; // in us
static const bool USE_SERIAL_MONITOR = false;
static const int MSG_LENGTH = 780;
# define LOGGING_UNDERSAMPLING  1

/***************************************************************************************************/
/********************************************* pwm_lib *********************************************/
/***************************************************************************************************/
using namespace arduino_due::pwm_lib;

//#define PWM_FREQUENCY 10000
//#define PWM_PERIOD_PIN_35 1000 // 10000 x (1e-8 secs) = 1e-4 sec
//#define PWM_DUTY_PIN_35 500

#define PWM_FREQUENCY 10000
#define PWM_PERIOD_PIN_35 10000 // 10000 x (1e-8 secs) = 1e-4 sec
#define PWM_DUTY_PIN_35 0

// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWMH0_PC3> pwm_pin35;
pwm_wrapper<
  decltype(pwm_pin35)
> pwm_wrapper_pin35(pwm_pin35);
pwm_base* pwm_wrapper_pin35_ptr=&pwm_wrapper_pin35;

/***************************************************************************************************/
/******************************* proximal flow sensor calibration **********************************/
/***************************************************************************************************/
static const float coefficient_dP2flow = 0.6438;
static const float coefficient_dp2flow_offset = 1.1029;

/***************************************************************************************************/
/***************************************************************************************************/
/***************************************************************************************************/
static const int CMD_LENGTH = 4;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

// mode def
# define MODE_OFF     0
# define MODE_VC_AC   1
# define MODE_PC_AC   2
# define MODE_VC_SIMV 3
# define MODE_PC_SIMV 4
# define MODE_PSV     5

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
static const uint8_t CMD_MODE = 9;
static const uint8_t CMD_CLOSE_VALVE = 10;
static const uint8_t CMD_STEPPER_CONTROL_AIR = 11;
static const uint8_t CMD_STEPPER_CONTROL_OXYGEN = 12;
static const uint8_t CMD_SET_BIAS_FLOW = 13;
static const uint8_t CMD_Trigger_th = 14;
static const uint8_t CMD_ONOFF = 15;
static const uint8_t CMD_Exhalation_Control_RiseTime = 16;

// full scale values
static const float flow_FS = 200;
static const float volume_FS = 1500;
static const float paw_FS = 100;
static const float psupply_FS = 30;
static const float Ti_FS = 5;
static const float Vt_FS = 1500;
static const float PEEP_FS = 30;
static const float RR_FS = 60;
static const float valve_pos_open_steps_FS = -125;
static const float pc_rise_time_ms_FS = 1000;
static const float PID_coefficient_P_FS = 0.1;
static const float PID_coefficient_I_frac_FS = 1;

// set parameters and their default values
int mode = MODE_PC_AC;
float RR = 18;
float Ti = 1;
float time_inspiratory_ms = Ti * 1000;
float Vt = 250;
float PEEP = 0;
float paw_trigger_th = -1.5;
float pc_rise_time_ms = 200;
float pinsp_setpoint = 20;
float psupport = 10;
bool pressure_support_enabled = false;
float valve_pos_open_steps = -100;

// measured variables
float dP = 0; 
float mflow_proximal = 0;
float mflow_air = 0;
float mflow_oxygen = 0;
float mflow_total = 0;
float mpexhalation = 0;
float mppatient = 0;
float mpaw = 0;
float mp_air = 0;
float mp_oxygen = 0;
volatile float mflow_peak = 0;
volatile float mvolume = 0;
volatile float mPEEP = 0;

// breathing control
float cycle_period_ms = 0; // duration of each breathing cycle
volatile float cycle_time_ms = 0;  // current time in the breathing cycle
volatile float time_ms_into_exhalation = 0;

// PID pressure control
float PID_coefficient_P = 0.01;
float PID_coefficient_I = 0.001;
float PID_coefficient_I_frac = 0.1;
float PID_coefficient_D = 0;
volatile float paw_setpoint_rise = 0;
volatile float paw_setpoint = 0;
volatile float PID_Insp_Integral = 0;
volatile float PID_Insp_Prop = 0;
volatile float paw_error = 0;

float PID_coefficient_P_exhalation_control = 0.02;
float PID_coefficient_I_exhalation_control = 0.005;
float rise_time_ms_exhalation_control = 0; 
volatile float p_exhalation_control_setpoint_rise = 0;
volatile float PID_exhalation_control_Integral = 0;
volatile float PID_exhalation_control_Prop = 0;
volatile float p_exhalation_control_error = 0;

// valve control
volatile float valve_opening = 0; // 0 - 1
volatile float valve_exhalation_control_closing = 0;

bool flag_close_valve_air_in_progress = false;
bool flag_valve_air_flow_detected = false;
bool flag_valve_air_close_position_reset = false;

bool flag_close_valve_oxygen_in_progress = false;
bool flag_valve_oxygen_flow_detected = false;
bool flag_valve_oxygen_close_position_reset = false;

volatile bool flag_log_data = false;
volatile bool flag_read_sensor = false;

// breathing control related flags
volatile bool is_breathing = false;
volatile bool is_in_inspiratory_phase = false;
volatile bool is_in_pressure_regulation = false;
volatile bool is_in_expiratory_phase = false;
volatile bool is_in_pressure_regulation_rise = false;
volatile bool is_in_pressure_regulation_plateau = false;
volatile bool is_in_pressure_support = false;
volatile bool PEEP_is_reached = false;

// data logging
volatile int counter_log_data = 0;

// other variables
uint16_t tmp_uint16;
int16_t tmp_int16;
long tmp_long;
volatile uint32_t timestamp = 0; // in number of TIMER_PERIOD_us

/***************************************************************************************************/
/********************************************* sensors *********************************************/
/***************************************************************************************************/

SFM3000 sfm3000_1;
SFM3000 sfm3000_2;
SDP8XXSensor sdp;

// create Honeywell_ABP instance
// refer to datasheet for parameters
Honeywell_ABP abp_30psi_1(
  0x28,   // I2C address
  0,      // minimum pressure
  30,      // maximum pressure
  "psi"   // pressure unit
);
Honeywell_ABP abp_30psi_2(
  0x28,   // I2C address
  0,      // minimum pressure
  30,      // maximum pressure
  "psi"   // pressure unit
);
Honeywell_ABP abp_1psi_1(
  0x28,   // I2C address
  0,      // minimum pressure
  1,      // maximum pressure
  "psi"   // pressure unit
);
Honeywell_ABP abp_1psi_2(
  0x28,   // I2C address
  0,      // minimum pressure
  1,      // maximum pressure
  "psi"   // pressure unit
);
Honeywell_ABP abp_1psi_3(
  0x28,   // I2C address
  0,      // minimum pressure
  1,      // maximum pressure
  "psi"   // pressure unit
);

uint8_t cmd[1];
int ret_sfm3000_1 = 0;
int ret_sfm3000_2 = 0;

/***************************************************************************************************/
/******************************************* stepper ***********************************************/
/***************************************************************************************************/
// stepper
static const int UART_CS_S0 = 46;
static const int UART_CS_S1 = 47;
#define STEPPER_SERIAL Serial3
static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;
TMC2209Stepper Z_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
static const int Z_dir = 28;
static const int Z_step = 26;
static const int Z_N_microstepping = 2;
static const long steps_per_mm_Z = 30*Z_N_microstepping; 
//constexpr float MAX_VELOCITY_Z_mm = 25; 
//constexpr float MAX_ACCELERATION_Z_mm = 2000; // 12.5 ms to reach max speed
constexpr float MAX_VELOCITY_Z_mm = 25; 
constexpr float MAX_ACCELERATION_Z_mm = 1600; // 12.5 ms to reach max speed
AccelStepper stepper_Z = AccelStepper(AccelStepper::DRIVER, Z_step, Z_dir);

/***************************************************************************************************/
/******************************************* setup *************************************************/
/***************************************************************************************************/
void setup() {

  // starting PWM signals
  pwm_wrapper_pin35_ptr->start(PWM_PERIOD_PIN_35,PWM_DUTY_PIN_35);

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established
  buffer_rx_ptr = 0;

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // tmp gnd pin for the SDP810 sensor
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);

  Wire.setClock(400000);
  Wire.begin();

  // initialize the SFM sensor 1
  enableMuxPort(0);
  while(true) 
  {
    int ret_sfm3000_1 = sfm3000_1.init();
    if (ret_sfm3000_1 == 0) 
    {
      if(USE_SERIAL_MONITOR)
        SerialUSB.print("init() for sensor 1: success\n");
      break;
    } 
    else 
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init() for sensor 1: failed, ret = ");
        SerialUSB.println(ret_sfm3000_1);
      }
      delay(1000);
    }
  }
  // get scale and offset factor for the SFM sensor
  sfm3000_1.get_scale_offset();
  sfm3000_2.start_continuous();

  enableMuxPort(1);
  while(true) 
  {
    int ret_sfm3000_2 = sfm3000_2.init();
    if (ret_sfm3000_2 == 0) 
    {
      if(USE_SERIAL_MONITOR)
        SerialUSB.print("init() for sensor 2: success\n");
      break;
    } 
    else 
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init() for sensor 2: failed, ret = ");
        SerialUSB.println(ret_sfm3000_2);
      }
      delay(1000);
    }
  }
  // get scale and offset factor for the SFM sensor
  sfm3000_2.get_scale_offset();
  sfm3000_2.start_continuous();

  // initialize the SDP sensor
  sdp.stopContinuous(); // stop continuous measurement if it's running
  while (true)
  {
    int ret = sdp.init();
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
  sdp.startContinuous(true);
  sdp.startContinuousWait(true);

  // stepper driver init.
  pinMode(Z_dir, OUTPUT);
  pinMode(Z_step, OUTPUT);
  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);
  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);
  delayMicroseconds(1000);
  
  select_driver(1);
  while(!STEPPER_SERIAL);
  Z_driver.begin();
  Z_driver.I_scale_analog(false);  
  Z_driver.rms_current(350,0.2); //I_run and holdMultiplier
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

  if (is_breathing)
  {
    
    // update cycle timer
    cycle_time_ms = cycle_time_ms + TIMER_PERIOD_us / 1000;
    
    if (mode == MODE_PC_AC || mode == MODE_PSV)
    {
      if (mode == MODE_PC_AC)
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
          mflow_peak = 0;
          mPEEP = mpaw;
          set_valve2_closing(1);
          digitalWrite(13, HIGH);
          PID_Insp_Integral = 0;
          //is_in_pressure_support = false; // for testing pressure support only
        }
      }
      // for the first test only support PC-CMV and PSV
      if(mode == MODE_PSV)
      {
        // patient triggered breath
        if ( mpaw < PEEP + paw_trigger_th && is_in_expiratory_phase && is_in_inspiratory_phase == false )
        {
          cycle_time_ms = 0;
          is_in_inspiratory_phase = true;
          is_in_pressure_regulation_rise = true;
          is_in_expiratory_phase = false;
          PEEP_is_reached = false;
          mvolume = 0;
          mflow_peak = 0;
          set_valve2_closing(1);
          digitalWrite(13, HIGH);
          PID_Insp_Integral = 0;
          is_in_pressure_support = true;
          
          // determine whether this is a mandatory breath or a supported breath
          // for testing, set all patient triggered breathes as supported breathes
          //if(pressure_support_enabled == true)
          //  is_in_pressure_support = true;
        }
      }
      
      if(is_in_pressure_regulation_rise == true && cycle_time_ms <= time_inspiratory_ms)
      {
        // update mflow_peak
        mflow_peak =  max(mflow_proximal,mflow_peak);
        // determine state and calculate setpoint
        // paw_setpoint = is_in_pressure_support ? (mPEEP + psupport) : pinsp_setpoint;
        paw_setpoint = pinsp_setpoint;
        // determine state
        if(cycle_time_ms < pc_rise_time_ms + 200 && mpaw < 0.95*paw_setpoint && is_in_pressure_regulation_plateau == false)
          is_in_pressure_regulation_rise = true;
        else
        {
          is_in_pressure_regulation_rise = false;
          is_in_pressure_regulation_plateau = true;
        }
        // determine rise setpoint
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
  
      if(is_in_pressure_regulation_plateau)
      {
        // calculate error
        paw_error = pinsp_setpoint - mpaw;
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

      // advanced termination
      if(is_in_pressure_support && mflow_proximal <= 0.25*mflow_peak && is_in_pressure_regulation_plateau && is_in_expiratory_phase == false)
      {
        // change to exhalation
        is_in_inspiratory_phase = false;
        is_in_pressure_regulation_plateau = false;
        is_in_pressure_support = false;
        is_in_expiratory_phase = true;
        valve_opening = 0;
        set_valve1_pos(valve_opening);
        set_valve2_closing(0);
        digitalWrite(13, LOW);
      }
    }
    
    // volume control
    else
    {
      // time-triggered breath
      if (cycle_time_ms > cycle_period_ms && is_in_inspiratory_phase == false )
      {
        cycle_time_ms = 0;
        is_in_inspiratory_phase = true;
        is_in_expiratory_phase = false;
        PEEP_is_reached = false;
        mvolume = 0;
        mflow_peak = 0;
        mPEEP = mpaw;
        set_valve2_closing(1);
        valve_opening = 0.36;
        set_valve1_pos(valve_opening);
        digitalWrite(13, HIGH);
      }
      if (mvolume >= Vt && is_in_inspiratory_phase == true)
      {
        valve_opening = 0;
        set_valve1_pos(valve_opening);
        is_in_inspiratory_phase = false;
      }
    }    

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

    // breathing control - change to exhalation when Ti is reached
    // if (cycle_time_ms > time_inspiratory_ms && is_in_expiratory_phase == false)
    if (cycle_time_ms > time_inspiratory_ms && is_in_expiratory_phase == false)
    {
      is_in_inspiratory_phase = false;
      is_in_pressure_regulation_rise = false;
      is_in_pressure_regulation_plateau = false;
      is_in_expiratory_phase = true;
      valve_opening = 0;
      set_valve1_pos(valve_opening);
      set_valve2_closing(0);
      digitalWrite(13, LOW);

      // exhalation control
      time_ms_into_exhalation = 0;
      valve_exhalation_control_closing = 0;
      PID_exhalation_control_Integral = 0;
      p_exhalation_control_setpoint_rise = 0;
    }

    if (is_in_expiratory_phase == true)
    { 
      // for debugging, when rise_time_ms_exhalation_control is set to 0, do open loop control of exhalation control pressure
      if (rise_time_ms_exhalation_control > 0)
      {
        time_ms_into_exhalation = time_ms_into_exhalation + TIMER_PERIOD_us / 1000;
        // calculate setpoint
        p_exhalation_control_setpoint_rise = time_ms_into_exhalation > rise_time_ms_exhalation_control? PEEP : PEEP * (time_ms_into_exhalation/rise_time_ms_exhalation_control);
        // calculate error
        p_exhalation_control_error = p_exhalation_control_setpoint_rise - mpexhalation;
        PID_exhalation_control_Integral = PID_exhalation_control_Integral + PID_coefficient_I_exhalation_control*p_exhalation_control_error;
        PID_exhalation_control_Integral = PID_exhalation_control_Integral > 1 ? 1 : PID_exhalation_control_Integral;
        PID_exhalation_control_Integral = PID_exhalation_control_Integral < 0 ? 0 : PID_exhalation_control_Integral;
        PID_Insp_Prop = PID_coefficient_P_exhalation_control*PID_coefficient_I_exhalation_control;
        // generate command
        valve_exhalation_control_closing = PID_Insp_Prop + PID_exhalation_control_Integral;
        valve_exhalation_control_closing = valve_exhalation_control_closing > 1 ? 1 : valve_exhalation_control_closing;
        valve_exhalation_control_closing = valve_exhalation_control_closing < 0 ? 0 : valve_exhalation_control_closing;
        set_valve2_closing(valve_exhalation_control_closing);
      }
      else
        set_valve2_closing(PEEP/PEEP_FS);
    }
  }

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
      {
        PEEP = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*PEEP_FS;
        // when debugging (is_breathing set to 0), set exhalation valve closing to PEEP/PEEP_FS
        if (is_breathing == false)
          set_valve2_closing(PEEP/PEEP_FS);
      }
      else if (buffer_rx[0] == CMD_Flow)
        valve_pos_open_steps = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * valve_pos_open_steps_FS;
      else if (buffer_rx[0] == CMD_Pinsp)
        pinsp_setpoint =  ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * paw_FS;
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
      else if (buffer_rx[0] == CMD_MODE)
        mode = buffer_rx[1];
      else if (buffer_rx[0] == CMD_CLOSE_VALVE)
      {
        if (buffer_rx[1] == 0)
          flag_close_valve_air_in_progress = true; // close the air valve
        // @TODO: close oxygen valve
      }
      else if (buffer_rx[0] == CMD_STEPPER_CONTROL_AIR)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        stepper_Z.runToNewPosition(stepper_Z.currentPosition()+relative_position);
      }
      else if (buffer_rx[0] == CMD_SET_BIAS_FLOW)
        stepper_Z.setCurrentPosition(0);
      else if (buffer_rx[0] == CMD_Trigger_th)
        paw_trigger_th =  - ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * paw_FS;
      else if (buffer_rx[0] == CMD_ONOFF)
        is_breathing = buffer_rx[1];
      else if (buffer_rx[0] == CMD_Exhalation_Control_RiseTime)
        rise_time_ms_exhalation_control =  ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * pc_rise_time_ms_FS;
    }
  }

  if (flag_read_sensor)
  {
    enableMuxPort(0);
    int ret_sfm3000_1 = sfm3000_1.read_sample();
    enableMuxPort(1);
    int ret_sfm3000_2 = sfm3000_2.read_sample();
    enableMuxPort(2);
    abp_30psi_1.update();
    enableMuxPort(3);
    abp_30psi_2.update();
    enableMuxPort(4);
    abp_1psi_1.update();
    enableMuxPort(5);
    abp_1psi_2.update();
    enableMuxPort(6);
    abp_1psi_3.update();

    mpexhalation = abp_1psi_1.pressure()*70.307;
    mppatient = abp_1psi_2.pressure()*70.307;
    mpaw = abp_1psi_3.pressure()*70.307;

    mp_air = abp_30psi_1.pressure();
    mp_oxygen = abp_30psi_2.pressure();

    if (ret_sfm3000_1 == 0)
      mflow_air = sfm3000_1.get_flow();
    if (ret_sfm3000_2 == 0)
      mflow_oxygen = sfm3000_2.get_flow();
    mflow_total = mflow_air + mflow_oxygen;
    
    if (sdp.readContinuous() == 0)
      dP = sdp.getDifferentialPressure();
    if(abs(dP)<0.3)
      mflow_proximal = 0;
    else if(dP >=3 )
      mflow_proximal = dP * coefficient_dP2flow + coefficient_dp2flow_offset;
    else
      mflow_proximal = dP * coefficient_dP2flow - coefficient_dp2flow_offset;
    
    mvolume = mvolume + mflow_proximal * 1000 * (float(TIMER_PERIOD_us) / 1000000 / 60);
    flag_read_sensor = false;
  }

  if (flag_close_valve_air_in_progress && ret_sfm3000_1 == 0)
  {
    if (mflow_air >= 0.2)
    {
      flag_valve_air_flow_detected = true;
      stepper_Z.runToNewPosition(stepper_Z.currentPosition() + 1);
    }
    else
    {
      if (flag_valve_air_flow_detected)
      {
        stepper_Z.runToNewPosition(stepper_Z.currentPosition() + 1);
        stepper_Z.setCurrentPosition(0);
        flag_close_valve_air_in_progress = false;
        flag_valve_air_flow_detected = false;
        flag_valve_air_close_position_reset = true;
      }
      else
      {
        flag_close_valve_air_in_progress = false;
        flag_valve_air_flow_detected = false;
        flag_valve_air_close_position_reset = false;
      }
    }
  }

  if (flag_log_data)
  {
    flag_log_data = false;
    
    // field 1: time
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 24);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 16);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp %256);

    // field 2: motor position - air valve
    tmp_uint16 = signed2NBytesUnsigned(stepper_Z.currentPosition(), 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 3: motor position - oxygen valve
    // tmp_uint16 = signed2NBytesUnsigned(stepper_Z.currentPosition(), 2);
    tmp_uint16 = 0;
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 4 flow - air
    tmp_long = (65536 / 2) * mflow_air / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 5 flow - oxygen
    tmp_long = (65536 / 2) * mflow_oxygen / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 6 flow - proximal
    tmp_long = (65536 / 2) * mflow_proximal / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 7 upstream pressure - air 
    tmp_long = (65536 / 2) * mp_air / psupply_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 8 upstream pressure - oxygen 
    tmp_long = (65536 / 2) * mp_oxygen / psupply_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 9 exhalation control valve pressure
    tmp_long = (65536 / 2) * mpexhalation / paw_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 10 patient pressure
    tmp_long = (65536 / 2) * mppatient / paw_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 11 airway pressure
    tmp_long = (65536 / 2) * mpaw / paw_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 12 volume
    tmp_long = (65536 / 2) * mvolume / volume_FS;;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 13 reserved for FiO2
    tmp_long = 0;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 14 reserved for humidity
    tmp_long = 0;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    if (buffer_tx_ptr == MSG_LENGTH)
    {
      buffer_tx_ptr = 0; // this was missing
      if(USE_SERIAL_MONITOR)
      {
          SerialUSB.print(mpaw);
          SerialUSB.print("\t ");
          SerialUSB.print(mflow_proximal);
          SerialUSB.print("\t ");
          SerialUSB.println(mvolume);
      }
      else
        SerialUSB.write(buffer_tx, MSG_LENGTH);
    }
  }

  // run stepper
  stepper_Z.run();

}

/***************************************************************************************************/
/********************************************* valves **********************************************/
/***************************************************************************************************/
void set_valve1_pos(float pos)
{
  stepper_Z.moveTo(valve_pos_open_steps * pos);
}

float get_valve1_pos()
{
  return stepper_Z.currentPosition();
}

/*
void set_valve2_state(int state)
{
  
  // close the valve
  if (state == 0) 
    change_duty(*pwm_wrapper_pin35_ptr,EXHALATION_CONTROL_DUTY_CLOSE,PWM_PERIOD_PIN_35);
  else
  // control PEEP
     change_duty(*pwm_wrapper_pin35_ptr,exhalation_control_PEEP_duty,PWM_PERIOD_PIN_35);
}
*/
void set_valve2_closing(float closing)
{
  float duty_cycle_fraction = 0.3 + closing;
  if (duty_cycle_fraction > 0.999)
    duty_cycle_fraction = 0.999;
  change_duty(*pwm_wrapper_pin35_ptr,duty_cycle_fraction*PWM_PERIOD_PIN_35,PWM_PERIOD_PIN_35);
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

/***************************************************************************************************/
/*********************************************  utils  *********************************************/
/***************************************************************************************************/
//Enables a specific port number
boolean enableMuxPort(byte portNumber)
{
  byte settings = (1 << portNumber);
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();
  return(true);
}

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
