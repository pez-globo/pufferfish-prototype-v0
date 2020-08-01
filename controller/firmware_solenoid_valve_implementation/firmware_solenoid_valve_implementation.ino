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
static const uint32_t EXHALATION_CONTROL_DUTY_CLOSE = 9000;
uint32_t exhalation_control_PEEP_duty = 5000;
static const float TIMER_PERIOD_us = 1500; // in us
static const bool USE_SERIAL_MONITOR = false; // for debug
static const int MSG_LENGTH = 33*26;
# define LOGGING_UNDERSAMPLING  1

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

// defining pwm object using pin 37, pin PC5 mapped to pin 37 on the DUE
// this object uses PWM channel 1
pwm<pwm_pin::PWMH1_PC5> pwm_pin37;
pwm_wrapper<decltype(pwm_pin37)> pwm_wrapper_pin37(pwm_pin37);
pwm_base* pwm_wrapper_pin37_ptr=&pwm_wrapper_pin37;

// defining pwm object using pin 39, pin PC7 mapped to pin 39 on the DUE
// this object uses PWM channel 2
pwm<pwm_pin::PWMH2_PC7> pwm_pin39;
pwm_wrapper<decltype(pwm_pin39)> pwm_wrapper_pin39(pwm_pin39);
pwm_base* pwm_wrapper_pin39_ptr=&pwm_wrapper_pin39;

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
static const uint8_t CMD_CLOSE_VALVE_AIR = 10;
static const uint8_t CMD_VALVE_CONTROL_AIR = 11;
static const uint8_t CMD_VALVE_CONTROL_OXYGEN = 12;
static const uint8_t CMD_SET_BIAS_FLOW_AIR = 13;
static const uint8_t CMD_Trigger_th = 14;
static const uint8_t CMD_ONOFF = 15;
static const uint8_t CMD_Exhalation_Control_RiseTime = 16;
static const uint8_t CMD_CLOSE_VALVE_OXYGEN = 17;
static const uint8_t CMD_SET_BIAS_FLOW_OXYGEN = 18;
static const uint8_t CMD_SET_FIO2 = 19;

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
static const float dP_FS = 500;

// parameters
static const float exhalation_flow_no_trigger_threshold = -20;

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
float fio2 = 0.50;
bool pressure_support_enabled = false;
float valve_pos_open_steps = -100;
float valve_opening_vc = 0.4;

float p_exhalation_control_target = 0; // the pressure applied to the membrane is different from the target PEEP
// p_exhalation_control_target = 0.4799 * PEEP - 0.2744

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
volatile float mvolume_oxygen = 0;
volatile float mvolume_air = 0;
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
float rise_time_ms_exhalation_control = 400; 
volatile float p_exhalation_control_setpoint_rise = 0;
volatile float PID_exhalation_control_Integral = 0;
volatile float PID_exhalation_control_Prop = 0;
volatile float p_exhalation_control_error = 0;

// valve control
volatile float valve_opening = 0; // 0 - 1
volatile float valve_opening_air = 0; // 0 - 1
volatile float valve_opening_oxygen = 0; // 0 - 1
volatile float valve_exhalation_control_closing = 0;

float valve_opening_air_bias = 0;
float valve_opening_oxygen_bias = 0;

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
Honeywell_ABP abp_60psi_1(
  0x28,   // I2C address
  0,      // minimum pressure
  60,      // maximum pressure
  "psi"   // pressure unit
);
Honeywell_ABP abp_60psi_2(
  0x28,   // I2C address
  0,      // minimum pressure
  60,      // maximum pressure
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
/******************************************* setup *************************************************/
/***************************************************************************************************/
void setup() {

  // starting PWM signals
  pwm_wrapper_pin35_ptr->start(PWM_PERIOD,PWM_DUTY);
  pwm_wrapper_pin37_ptr->start(PWM_PERIOD,PWM_DUTY);
  pwm_wrapper_pin39_ptr->start(PWM_PERIOD,PWM_DUTY);

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
        SerialUSB.print("init() for sensor SFM3000 - air: success\n");
      break;
    } 
    else 
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init() for sensor SFM3000 - air: failed, ret = ");
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
        SerialUSB.print("init() for sensor SFM3000 - oxygen: success\n");
      break;
    } 
    else 
    {
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.print("init() for sensor SFM3000 - oxygen: failed, ret = ");
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
          mvolume_air = 0;
          mvolume_oxygen = 0;
          mflow_peak = 0;
          mPEEP = mpaw;
          valve_exhalation_control_closing = 1;
          set_valve2_closing(valve_exhalation_control_closing);
          digitalWrite(13, HIGH);
          PID_Insp_Integral = 0;
        }
        // patient triggered breath
        if ( mpaw < PEEP + paw_trigger_th && is_in_expiratory_phase && is_in_inspiratory_phase == false && mflow_proximal > exhalation_flow_no_trigger_threshold  )
        {
          cycle_time_ms = 0;
          is_in_inspiratory_phase = true;
          is_in_pressure_regulation_rise = true;
          is_in_expiratory_phase = false;
          PEEP_is_reached = false;
          mvolume = 0;
          mvolume_air = 0;
          mvolume_oxygen = 0;
          mflow_peak = 0;
          valve_exhalation_control_closing = 1;
          set_valve2_closing(valve_exhalation_control_closing);
          digitalWrite(13, HIGH);
          PID_Insp_Integral = 0;
        }
      }

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
          mvolume_air = 0;
          mvolume_oxygen = 0;
          mflow_peak = 0;
          valve_exhalation_control_closing = 1;
          set_valve2_closing(valve_exhalation_control_closing);
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
        valve_opening_air = (1-fio2)/0.77 * valve_opening;
        valve_opening_oxygen = (1-((1-fio2)/0.77)) * valve_opening;
        set_valve_opening_air(valve_opening_air);
        set_valve_opening_oxygen(valve_opening_oxygen);
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
        valve_opening_air = (1-fio2)/0.77 * valve_opening;
        valve_opening_oxygen = (1-((1-fio2)/0.77)) * valve_opening;
        set_valve_opening_air(valve_opening_air);
        set_valve_opening_oxygen(valve_opening_oxygen);
      }

      // advanced termination
      if(is_in_pressure_support && mflow_proximal <= 0.25*mflow_peak && is_in_pressure_regulation_plateau && is_in_expiratory_phase == false)
      {
        // change to exhalation
        is_in_inspiratory_phase = false;
        is_in_pressure_regulation_plateau = false;
        is_in_pressure_support = false;
        is_in_expiratory_phase = true;
        valve_opening_air = 0;
        set_valve_opening_air(valve_opening_air);
        valve_opening_oxygen = 0;
        set_valve_opening_oxygen(valve_opening_oxygen);
        
        time_ms_into_exhalation = 0;
        valve_exhalation_control_closing = 0;
        PID_exhalation_control_Integral = 0;
        p_exhalation_control_setpoint_rise = 0;
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
        mvolume_air = 0;
        mvolume_oxygen = 0;
        mflow_peak = 0;
        mPEEP = mpaw;
        valve_exhalation_control_closing = 1;
        set_valve2_closing(valve_exhalation_control_closing);
        valve_opening_air = (1-fio2)/0.77 * valve_opening_vc;
        valve_opening_oxygen = (1-((1-fio2)/0.77)) * valve_opening_vc;
        set_valve_opening_air(valve_opening_air);
        set_valve_opening_oxygen(valve_opening_oxygen);
        
        digitalWrite(13, HIGH);
      }

      // patient triggered breath
      if ( mpaw < PEEP + paw_trigger_th && is_in_expiratory_phase && is_in_inspiratory_phase == false && mflow_proximal > exhalation_flow_no_trigger_threshold )
      {
        cycle_time_ms = 0;
        is_in_inspiratory_phase = true;
        is_in_expiratory_phase = false;
        PEEP_is_reached = false;
        mvolume = 0;
        mvolume_air = 0;
        mvolume_oxygen = 0;
        mflow_peak = 0;
        valve_exhalation_control_closing = 1;
        set_valve2_closing(valve_exhalation_control_closing);
        valve_opening_air = (1-fio2)/0.77 * valve_opening_vc;
        valve_opening_oxygen = (1-((1-fio2)/0.77)) * valve_opening_vc;
        set_valve_opening_air(valve_opening_air);
        set_valve_opening_oxygen(valve_opening_oxygen);
        digitalWrite(13, HIGH);
      }
        
      // stop the inspiratory flow when the set tidal volume is delivered
      if (mvolume >= Vt && is_in_inspiratory_phase == true)
      {
        valve_opening_air = 0;
        set_valve_opening_air(valve_opening_air);
        valve_opening_oxygen = 0;
        set_valve_opening_oxygen(valve_opening_oxygen);
        is_in_inspiratory_phase = false;
      }
    }    

    // breathing control - change to exhalation when Ti is reached
    // if (cycle_time_ms > time_inspiratory_ms && is_in_expiratory_phase == false)
    if (cycle_time_ms > time_inspiratory_ms && is_in_expiratory_phase == false)
    {
      // set state variables
      is_in_inspiratory_phase = false;
      is_in_pressure_regulation_rise = false;
      is_in_pressure_regulation_plateau = false;
      is_in_expiratory_phase = true;

      // stop inspiratory flow
      valve_opening_air = 0;
      set_valve_opening_air(valve_opening_air);
      valve_opening_oxygen = 0;
      set_valve_opening_oxygen(valve_opening_oxygen);
      
      // turn off LED
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
        p_exhalation_control_setpoint_rise = time_ms_into_exhalation > rise_time_ms_exhalation_control? p_exhalation_control_target : p_exhalation_control_target * (time_ms_into_exhalation/rise_time_ms_exhalation_control);
        // calculate error
        p_exhalation_control_error = p_exhalation_control_setpoint_rise - mpexhalation;
        PID_exhalation_control_Integral = PID_exhalation_control_Integral + PID_coefficient_I_exhalation_control*p_exhalation_control_error;
        PID_exhalation_control_Integral = PID_exhalation_control_Integral > 1 ? 1 : PID_exhalation_control_Integral;
        PID_exhalation_control_Integral = PID_exhalation_control_Integral < 0 ? 0 : PID_exhalation_control_Integral;
        PID_exhalation_control_Prop = PID_coefficient_P_exhalation_control*PID_coefficient_I_exhalation_control;
        // generate command
        valve_exhalation_control_closing = PID_exhalation_control_Prop + PID_exhalation_control_Integral;
        valve_exhalation_control_closing = valve_exhalation_control_closing > 1 ? 1 : valve_exhalation_control_closing;
        valve_exhalation_control_closing = valve_exhalation_control_closing < 0 ? 0 : valve_exhalation_control_closing;
        set_valve2_closing(valve_exhalation_control_closing);
      }
      else
      {
        valve_exhalation_control_closing = p_exhalation_control_target;
        set_valve2_closing(valve_exhalation_control_closing);
      }
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
        p_exhalation_control_target = PEEP*0.4799-0.2744;
        p_exhalation_control_target = p_exhalation_control_target > 0 ? p_exhalation_control_target : 0;
        // when debugging (is_breathing set to 0), set exhalation valve closing to PEEP/PEEP_FS
        if (is_breathing == false)
        {
          valve_exhalation_control_closing = p_exhalation_control_target;
          set_valve2_closing(valve_exhalation_control_closing);
        }
      }
      //      else if (buffer_rx[0] == CMD_Flow)
      //        valve_pos_open_steps = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * valve_pos_open_steps_FS;
      else if (buffer_rx[0] == CMD_Flow)
        valve_opening_vc = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536);
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
      else if (buffer_rx[0] == CMD_CLOSE_VALVE_AIR)
      {
        if (buffer_rx[1] == 0)
          flag_close_valve_air_in_progress = true; // close the air valve
      }
      else if (buffer_rx[0] == CMD_CLOSE_VALVE_OXYGEN)
      {
        if (buffer_rx[1] == 0)
          flag_close_valve_oxygen_in_progress = true; // close the oxygen valve
      }
      else if (buffer_rx[0] == CMD_VALVE_CONTROL_AIR)
      {
        float valve_opening = uint16_t(buffer_rx[2])*256 + uint16_t(buffer_rx[3]);
        valve_opening = valve_opening/65535;
        set_valve_opening_air(valve_opening);
      }
      else if (buffer_rx[0] == CMD_VALVE_CONTROL_OXYGEN)
      {
        float valve_opening = uint16_t(buffer_rx[2])*256 + uint16_t(buffer_rx[3]);
        valve_opening = valve_opening/65535;
        set_valve_opening_oxygen(valve_opening);
      }
      else if (buffer_rx[0] == CMD_SET_BIAS_FLOW_AIR)
        valve_opening_air_bias = valve_opening_air;
      else if (buffer_rx[0] == CMD_SET_BIAS_FLOW_OXYGEN)
        valve_opening_oxygen_bias = valve_opening_oxygen;
      else if (buffer_rx[0] == CMD_Trigger_th)
        paw_trigger_th =  - ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * paw_FS;
      else if (buffer_rx[0] == CMD_ONOFF)
        is_breathing = buffer_rx[1];
      else if (buffer_rx[0] == CMD_Exhalation_Control_RiseTime)
        rise_time_ms_exhalation_control =  ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536) * pc_rise_time_ms_FS;
      else if (buffer_rx[0] == CMD_SET_FIO2)
        fio2 =  ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536);
    }
  }

  if (flag_read_sensor)
  {
    enableMuxPort(0);
    int ret_sfm3000_1 = sfm3000_1.read_sample();
    enableMuxPort(1);
    int ret_sfm3000_2 = sfm3000_2.read_sample();
    enableMuxPort(2);
    abp_60psi_1.update();
    enableMuxPort(3);
    abp_60psi_2.update();
    enableMuxPort(4);
    abp_1psi_1.update();
    enableMuxPort(5);
    abp_1psi_2.update();
    enableMuxPort(6);
    abp_1psi_3.update();

    mpexhalation = abp_1psi_1.pressure()*70.307;
    mppatient = abp_1psi_2.pressure()*70.307;
    mpaw = abp_1psi_3.pressure()*70.307;

    mp_air = abp_60psi_1.pressure();
    mp_oxygen = abp_60psi_2.pressure();

    if (ret_sfm3000_1 == 0)
      mflow_air = sfm3000_1.get_flow();
    if (ret_sfm3000_2 == 0)
      mflow_oxygen = sfm3000_2.get_flow()*142.8/140;
    mflow_total = mflow_air + mflow_oxygen;
    
    if (sdp.readContinuous() == 0)
      dP = sdp.getDifferentialPressure();

    // convert dP to flow (calibration coefficients need to be obtained for each sensor)
    if (dP>=100)
      mflow_proximal = -0.0004*dP*dP + 0.4266*dP + 14.6174;
    else if (dP<100 && dP>=3)
      mflow_proximal = -0.0005*dP*dP + 0.5618*dP + 2.3038;
    else if (dP<3 && dP>=0)
      mflow_proximal = -0.1909*dP*dP + 1.8461*dP + 0.0113;
    else if (dP<0 && dP>=-3)
      mflow_proximal = 0.0319*dP*dP + 1.0575*dP + -0.4434;
    else if (dP<-3 && dP>=-100)
      mflow_proximal = 0.0002*dP*dP + 0.5464*dP + -2.3002;
    else if (dP<-100)
      mflow_proximal = 0.0005*dP*dP + 0.4731*dP + -11.8645;
    if(abs(mflow_proximal) < 0.5)
      mflow_proximal = 0;
    /*
    if(abs(dP)<0.3)
      mflow_proximal = 0;
    else if(dP >=3 )
      mflow_proximal = dP * coefficient_dP2flow + coefficient_dp2flow_offset;
    else
      mflow_proximal = dP * coefficient_dP2flow - coefficient_dp2flow_offset;
    */
    
    mvolume = mvolume + mflow_proximal * 1000 * (float(TIMER_PERIOD_us) / 1000000 / 60);
    mvolume_air = mvolume_air + mflow_air * 1000 * (float(TIMER_PERIOD_us) / 1000000 / 60);
    mvolume_oxygen = mvolume_oxygen + mflow_oxygen * 1000 * (float(TIMER_PERIOD_us) / 1000000 / 60);
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

    // field 2: air valve opening
    tmp_uint16 = 65535*valve_opening_air;
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 3: oxygen valve opening
    tmp_uint16 = 65535*valve_opening_oxygen;
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
    tmp_long = (65536 / 2) * mvolume / volume_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    /*
    // field 13 reserved for FiO2 (for now repurpose for storing dP)
    tmp_long = (65536 / 2) * dP / dP_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);
    */
    
    // field 13 reserved for humidity (for now repurpose for storing volume_air)    
    tmp_long = (65536 / 2) * mvolume_air / volume_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 14 reserved for humidity (for now repurpose for storing volume_oxygen)    
    tmp_long = (65536 / 2) * mvolume_oxygen / volume_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // field 15 tidal volume measured with air and oxygen sensor
    // to pull from pinch valve implementation
    tmp_long = (65536 / 2) * 0 / volume_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

    // filed 16 number of alarms
    buffer_tx[buffer_tx_ptr++] = byte(0);

    if (buffer_tx_ptr == MSG_LENGTH)
    {
      buffer_tx_ptr = 0; // this was missing
      if(USE_SERIAL_MONITOR)
      {
        /*
        SerialUSB.print(mpaw);
        SerialUSB.print("\t ");
        SerialUSB.print(mflow_proximal);
        SerialUSB.print("\t ");
        SerialUSB.println(mvolume);
        */
      }
      else
        SerialUSB.write(buffer_tx, MSG_LENGTH);
    }
  }
  
}

/***************************************************************************************************/
/********************************************* valves **********************************************/
/***************************************************************************************************/
void set_valve_opening_air(float opening)
{
  float duty_cycle_fraction = (valve_opening_air_bias + opening)/(valve_opening_air_bias+1);
  if (duty_cycle_fraction > 0.999)
    duty_cycle_fraction = 0.999;
  change_duty(*pwm_wrapper_pin37_ptr,duty_cycle_fraction*PWM_PERIOD,PWM_PERIOD);
}

void set_valve_opening_oxygen(float opening)
{
  float duty_cycle_fraction = (valve_opening_oxygen_bias + opening)/(valve_opening_oxygen_bias + 1);
  if (duty_cycle_fraction > 0.999)
    duty_cycle_fraction = 0.999;
  change_duty(*pwm_wrapper_pin39_ptr,duty_cycle_fraction*PWM_PERIOD,PWM_PERIOD);
}

void set_valve2_closing(float closing)
{
  float duty_cycle_fraction = 0.3 + closing;
  if (duty_cycle_fraction > 0.999)
    duty_cycle_fraction = 0.999;
  change_duty(*pwm_wrapper_pin35_ptr,duty_cycle_fraction*PWM_PERIOD,PWM_PERIOD);
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
