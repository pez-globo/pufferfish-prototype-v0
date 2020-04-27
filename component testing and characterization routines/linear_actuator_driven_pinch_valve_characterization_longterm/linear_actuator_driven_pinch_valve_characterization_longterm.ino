#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
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

#include <DueTimer.h>
static const float TIMER_PERIOD_us = 2500; // in us

static inline int sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

static const int CMD_LENGTH = 2;
static const int MSG_LENGTH = 504;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

static const int pin_valve1 = 30;
static const int pin_valve2 = 31;

static const float flow_FS = 300;
static const float volume_FS = 800;
static const float paw_FS = 60;
static const float Ti_FS = 5;
static const float Vt_FS = 800;
static const float PEEP_FS = 30;
static const float RR_FS = 60;

static const uint8_t CMD_Vt = 0;
static const uint8_t CMD_Ti = 1;
static const uint8_t CMD_RR = 2;
static const uint8_t CMD_PEEP = 3;
static const uint8_t CMD_Flow = 4;
static const uint8_t CMD_FlowDeceleratingSlope = 5;
static const uint8_t CMD_valve1 = 10;
static const uint8_t CMD_valve2 = 11;

static const float coefficient_dP2flow = 1.66;

volatile float dP = 0;
volatile float flow = 0;
volatile float volume = 0;
volatile float paw = 0;

float RR = 24;
float Ti = 1.4;
float Vt = 300;
float PEEP = 5;
float paw_trigger_th = -3;

float cycle_period_ms = 0; // duration of each breathing cycle
float cycle_time_ms = 0;  // current time in the breathing cycle
float time_inspiratory_ms = Ti * 1000;
float frequency_send_data = 50;
float counter_send_data = 0;

volatile bool flag_send_data = false;
volatile bool flag_read_sensor = false;

volatile bool is_breathing = true;
volatile bool is_in_inspiratory_phase = false;
volatile bool is_in_expiratory_phase = false;
volatile bool PEEP_is_reached = false;
volatile bool is_semi_closed = false;

volatile float valve_opening_percentage = 0;
volatile float valve_actuation_rate = 0.0005; // 2000 cycles for valve-open to close and vice-versa

int num_cycle_open_close = 1/valve_actuation_rate;

// Rate at which valve is opened and closed. 0.5 ms between each timer event - inspiration lasts 1s => 2000 steps
static const long travel = 3; // linear actuator travel
volatile long int stepper_pos = 0;

uint16_t tmp_uint16;
int16_t tmp_int16;
long tmp_long;

uint16_t timebase = 0; // in number of TIMER_PERIOD_us
static const long DISPLAY_RANGE_S = 20;

// stepper
#define STEPPER_SERIAL Serial3
#include <AccelStepper.h>

static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;
TMC2209Stepper Y_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);

// for PL35L-024-VLB8
static const long steps_per_mm_XY = 120; 
constexpr float MAX_VELOCITY_Y_mm = 40; 
constexpr float MAX_ACCELERATION_Y_mm = 300; 
static const long Y_NEG_LIMIT_MM = -12;
static const long Y_POS_LIMIT_MM = 12;

static const int Y_dir = 34;
static const int Y_step = 35;
static const int Y_driver_uart = 25;
static const int Y_en = 36;
static const int Y_gnd = 37;

AccelStepper stepper_Y = AccelStepper(AccelStepper::DRIVER, Y_step, Y_dir);

// Limit-switch integration
#define LIMIT_1 32
// Direction of stepper movement based on valve closing/opening. 
#define OPEN 1
#define CLOSE -1   

bool limit_finding_in_progress = false, limit_finding_complete = true;
bool homing_in_progress = false, at_home = false, homing_procedure_complete = false;
long int cycles_since_last_homing = 0;
long int homing_cycles = 1000*num_cycle_open_close; // No:of interrupt cycles between homing runs. 
bool reached_limit_closed = false, reached_limit_open = false;

// Limit-switch state
bool lim_1_prev=false, lim_1 = false;

// Stepping Direction
int step_direction = OPEN;
// Step size during homing
int step_size = 100;

// Distance in mm from limit-switch position to valve-closed position (needs to be characterized for each valve design).
float distance_to_valve_closed = 5;

// Use this to find the amount of time taken to find the limit switch from valve 0 position. 
// Proposed idea is to use this as a diagnostic test. 
float cycle_find_limit_switch_time = 0;
bool sent_homing_data = false;
bool homing_at_startup_complete = false;

long Pos_switch_closed;
long Pos_switch_open;

void FindLimits()
{
  lim_1_prev = lim_1;
  
  // We may also want to debounce this signal.
  ReadLimitSwitchDigital();

  // If switch transitions from Open to Closed when stepping in OPEN direction. 
  if(!lim_1 && lim_1_prev && reached_limit_closed==false)
  {
      reached_limit_closed = true;
      // Store the position as one limit
      Pos_switch_closed = stepper_Y.currentPosition();
      // Switch direction to start closing the valve.
      step_direction = CLOSE;
  }
  
  // If switch transitions from Closed to Open when stepping in CLOSE direction. This is when the switch is released.
  if(lim_1 && !lim_1_prev && reached_limit_open==false && reached_limit_closed == true)
  {
      reached_limit_open = true;
      // Store the position as one limit
      Pos_switch_open = stepper_Y.currentPosition();
  }
  if(reached_limit_closed && reached_limit_open)
  {
    limit_finding_complete = true;
    limit_finding_in_progress = false;    
  }

  stepper_Y.move(step_direction*step_size);
  
}

void MoveToHome()
{ 
  if(homing_in_progress == false)
  {
    stepper_Y.moveTo(step_direction*int(steps_per_mm_XY*distance_to_valve_closed));
    homing_in_progress == true;
  }

  if(homing_in_progress == true && stepper_Y.distanceToGo()==0)
  {

    homing_procedure_complete = true;
    homing_in_progress = false;
    
    // Reset the current position as the valve-closed position.
    valve_opening_percentage = 0;
    cycles_since_last_homing = 0;
  }
}

void ReadLimitSwitchDigital()
{
  lim_1 = digitalRead(LIMIT_1);
}

void timer_interruptHandler()
{
  
  // read sensor value
  flag_read_sensor = true;
  
  // update cycle timer
  cycle_time_ms = cycle_time_ms + TIMER_PERIOD_us / 1000;

  if(limit_finding_in_progress == true && limit_finding_complete == false)
  {
    cycle_find_limit_switch_time = cycle_find_limit_switch_time + TIMER_PERIOD_us / 1000;
  }

  // update homing cycles counter
  cycles_since_last_homing++;

  // If it's time to home and the valve has closed after the last-breathing cycle.
  if((cycles_since_last_homing >= homing_cycles && valve_opening_percentage<=0 && limit_finding_in_progress == false && homing_in_progress == false) || homing_at_startup_complete == false)
  {
    // Start Limit finding and Homing procedure. 
    cycle_find_limit_switch_time = 0;
    
    limit_finding_in_progress = true;
    limit_finding_complete = false;
    homing_procedure_complete = false;
    sent_homing_data = false;
    step_direction = OPEN;
    homing_at_startup_complete = true;
  }

  if(homing_in_progress==false && limit_finding_in_progress==false)
  {
    // if valve is fully-closed, start opening the valve
    if (valve_opening_percentage<=0)
    {
      step_direction = OPEN;
      valve_opening_percentage = 0;
    }
    else if (valve_opening_percentage>=1)
    {
      step_direction = CLOSE;
      valve_opening_percentage = 1;
    }
    
    valve_opening_percentage = valve_opening_percentage + step_direction*valve_actuation_rate;
    stepper_Y.moveTo(-valve_opening_percentage * travel * steps_per_mm_XY);
  }
}

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established
  buffer_rx_ptr = 0;

  SPI.begin(); // start SPI communication
  pressure_sensor.begin(); // run sensor initialization

  Wire.begin();
  sdp.stopContinuous(); // stop continuous measurement if it's running
  // init sensirion sensor
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

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

  // stepper driver init.
  pinMode(Y_driver_uart, OUTPUT);
  pinMode(Y_dir, OUTPUT);
  pinMode(Y_step, OUTPUT);
  pinMode(Y_gnd, OUTPUT);
  digitalWrite(Y_gnd, LOW);

  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);

  digitalWrite(Y_driver_uart, true);
  while (!STEPPER_SERIAL);
  Y_driver.begin();
  Y_driver.I_scale_analog(false);
  Y_driver.rms_current(450); //I_run and holdMultiplier
  Y_driver.microsteps(4);
  Y_driver.pwm_autoscale(true);
  Y_driver.TPOWERDOWN(2);
  Y_driver.en_spreadCycle(true);
  Y_driver.toff(4);
  digitalWrite(Y_driver_uart, false);

  stepper_Y.setEnablePin(Y_en);
  stepper_Y.setPinsInverted(false, false, true);
  stepper_Y.setMaxSpeed(MAX_VELOCITY_Y_mm * steps_per_mm_XY);
  stepper_Y.setAcceleration(MAX_ACCELERATION_Y_mm * steps_per_mm_XY);
  stepper_Y.enableOutputs();

  homing_at_startup_complete = false;
  
}



void loop()
{
  
  //  while (SerialUSB.available())
  //  {
  //    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
  //    buffer_rx_ptr = buffer_rx_ptr + 1;
  //    if (buffer_rx_ptr == CMD_LENGTH)
  //    {
  //      buffer_rx_ptr = 0;
  //      if (buffer_rx[0] == CMD_Vt)
  //        Vt = ((256 * float(buffer_rx[1]) + float(buffer_rx[2])) / 65536) * Vt_FS;
  //      else if (buffer_rx[0] == CMD_Ti)
  //      {
  //        Ti = ((256 * float(buffer_rx[1]) + float(buffer_rx[2])) / 65536) * Ti_FS;
  //        time_inspiratory_ms = Ti * 1000;
  //      }
  //      else if (buffer_rx[0] == CMD_RR)
  //      {
  //        RR = ((256 * float(buffer_rx[1]) + float(buffer_rx[2])) / 65536) * RR_FS;
  //        cycle_period_ms = (60 / RR) * 1000;
  //      }
  //      else if (buffer_rx[0] == CMD_PEEP)
  //        PEEP = ((256 * float(buffer_rx[1]) + float(buffer_rx[2])) / 65536) * PEEP_FS;
  //    }
  //  }

  // Update the stepper position:
  stepper_pos = stepper_Y.currentPosition();
  
  if (flag_read_sensor)
  {
    if (pressure_sensor.readSensor() == 0)
      paw = pressure_sensor.pressure();
    if (sdp.readContinuous() == 0)
      dP = sdp.getDifferentialPressure();
    flow = dP * coefficient_dP2flow;
    flag_read_sensor = false;

    tmp_long = (65536 / 2) * flow / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);

    // Also send the time to find limit switch
    if(limit_finding_complete == true && homing_procedure_complete == true && sent_homing_data == false)
    { 
      // Insert code here to send cycle_find_limit_switch_time after each homing run. 

      sent_homing_data = true;
    }
    else
    {
      // If homing data has already been sent, then just send 0. 
    }

    buffer_tx[buffer_tx_ptr++] = stepper_pos;
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);
  }
  
  if(buffer_tx_ptr==MSG_LENGTH)
  {
    SerialUSB.write(buffer_tx, MSG_LENGTH);
    buffer_tx_ptr = 0;
  }

  // find limit position (valve open) at startup
  if(limit_finding_in_progress == true && limit_finding_complete == false )
      FindLimits();
  // run homing
  else if(limit_finding_complete == true && homing_procedure_complete == false)
  {
      // Ensure the valve is closing
      step_direction = CLOSE;
      // Move till you fully close the valve (home position)
      MoveToHome();
  }
  stepper_Y.run();
}

// utils
long signed2NBytesUnsigned(long signedLong, int N)
{
  long NBytesUnsigned = signedLong + pow(256L, N) / 2;
  return NBytesUnsigned;
}
