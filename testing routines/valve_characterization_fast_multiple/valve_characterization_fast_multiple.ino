// SFM3x00 connection
//    Brown:    2 (RJ45)  SDA
//    Orange:   8 (RJ45)  GND
//    Blue:     4 (RJ45)  VCC (5V)
//    Green:    6 (RJ45)  SCK

static const bool USE_SERIAL_MONITOR = false;

#include <Wire.h>
#include <sfm3x00.h>
#include "Honeywell_ABP.h"
#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>
#include <DueTimer.h>

/*******************************************************************
 ******************************* Timer *****************************
 *******************************************************************/

static const float TIMER_PERIOD_us = 1000; // in us
static const int timer_div = 1;
volatile int timer_div_counter = 0;
volatile uint32_t cycle_count = 0;

/*******************************************************************
 ************************** Valve Selection ************************
 *******************************************************************/
static const int N_valves = 1; 
volatile uint8_t active_valve_ID;
volatile int counter_valve_selection = 0;
int number_of_timer_cycles_per_valve = 100;
bool flag_cycling_selection_of_valve = true;


/*******************************************************************
 ******************************* Flags *****************************
 *******************************************************************/
float flow_th = 0.2;
volatile bool flag_read_sensor = true;
volatile bool flag_write_data = true;
volatile bool flag_close_valve_in_progress = false;
volatile bool flag_valve_flow_detected[N_valves] = {false};
volatile bool flag_valve_close_position_reset[N_valves] = {false};
volatile bool flag_valve_doing_cyclic_motion = false;
volatile bool flag_valve_stop_cyclic_motion_requested = false;
int valve_cyclic_motion_dir = 1;
int valve_cyclic_motion_step_size = 2;
long cyclic_motion_limit_valve = 125;

static const int LIMIT_WAIT_CYCLES = 10;
volatile int limit_wait_counter = 0;
volatile bool is_at_limit = false;

/*******************************************************************
 ****************************** I2C MUX ****************************
 *******************************************************************/
#define MUX_ADDR_SFM3000 0x70
#define MUX_ADDR_ABP 0x71

/*******************************************************************
 *************************** Communication *************************
 *******************************************************************/
static const int CMD_LENGTH = 4;
static const int MSG_LENGTH = 960;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

long tmp_long;
uint16_t tmp_uint16;
float flow_FS = 200;
float pressure_FS = 60;
float pressure_FS2 = 100;

/*******************************************************************
 ************************* STEPPER CONTROL *************************
 *******************************************************************/
static const int UART_CS_S0 = 46;
static const int UART_CS_S1 = 47;

#define STEPPER_SERIAL Serial3
static const float R_SENSE = 0.11f;
TMC2209Stepper Z_driver_0b00(&STEPPER_SERIAL, R_SENSE, 0b00);
TMC2209Stepper Z_driver_0b01(&STEPPER_SERIAL, R_SENSE, 0b01);
TMC2209Stepper Z_driver_0b10(&STEPPER_SERIAL, R_SENSE, 0b10);
TMC2209Stepper Z_driver_0b11(&STEPPER_SERIAL, R_SENSE, 0b11);

static const int Z_dir_0 = 28;
static const int Z_step_0 = 26;
static const int Z_dir_1 = 24;
static const int Z_step_1 = 22;
static const int Z_dir_2 = 23;
static const int Z_step_2 = 25;
static const int Z_dir_3 = 27;
static const int Z_step_3 = 29;

static const int Z_N_microstepping = 2;
static const long steps_per_mm_Z = 30*Z_N_microstepping; 
constexpr float MAX_VELOCITY_Z_mm = 25; 
constexpr float MAX_ACCELERATION_Z_mm = 2000;
static const long Z_NEG_LIMIT_MM = -12;
static const long Z_POS_LIMIT_MM = 12;

// to do: include more steppers 
AccelStepper stepper_Z[N_valves] = {AccelStepper(AccelStepper::DRIVER, Z_dir_0, Z_step_0)};
long Z_commanded_target_position = 0;
bool Z_commanded_movement_in_progress = false;

/*******************************************************************
 ***************************** SENSORS *****************************
 *******************************************************************/
SFM3000 sfm3000;
int ret_sfm3000;

Honeywell_ABP abp_30psi(
  0x28,   // I2C address
  0,      // minimum pressure
  30,      // maximum pressure
  "psi"   // pressure unit
);

float mFlow;
float mPressure;

/*******************************************************************
 *****************************  SETUP  *****************************
 *******************************************************************/
void setup() 
{
  // wait for USB to connect
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established

  // reset rx buffer pointer
  buffer_rx_ptr = 0;

  // init LED
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);

  // pin init
  pinMode(Z_dir_0, OUTPUT);
  pinMode(Z_step_0, OUTPUT);
  pinMode(Z_dir_1, OUTPUT);
  pinMode(Z_step_1, OUTPUT);
  pinMode(Z_dir_2, OUTPUT);
  pinMode(Z_step_2, OUTPUT);
  pinMode(Z_dir_3, OUTPUT);
  pinMode(Z_step_3, OUTPUT);

  // UART - not used in this version
  /*
  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);
  */

  STEPPER_SERIAL.begin(115200);
  // select_driver(4);
  while(!STEPPER_SERIAL);
  Z_driver_0b00.begin();
  Z_driver_0b00.I_scale_analog(false);  
  Z_driver_0b00.rms_current(300,0.2); //I_run and holdMultiplier
  Z_driver_0b00.microsteps(Z_N_microstepping);
  Z_driver_0b00.pwm_autoscale(true);
  Z_driver_0b00.TPOWERDOWN(2);
  Z_driver_0b00.en_spreadCycle(false);
  Z_driver_0b00.toff(4);

  for(int i=0;i++;i<N_valves)
  {
    stepper_Z[i].setPinsInverted(false, false, true);
    stepper_Z[i].setMaxSpeed(MAX_VELOCITY_Z_mm*steps_per_mm_Z);
    stepper_Z[i].setAcceleration(MAX_ACCELERATION_Z_mm*steps_per_mm_Z);
    stepper_Z[i].enableOutputs();
  }
  
  // start I2C
  Wire.setClock(400000);
  Wire.begin();
  
  // initialize the SFM sensor
  for(int i=0;i++;i<N_valves)
  {
    enableMuxPort_SFM3000(i);
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
  }

  // delay
  delayMicroseconds(500000);

  // start the timer
  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);
  
}

/*******************************************************************
 *****************************  LOOPS  *****************************
 *******************************************************************/

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
      if(buffer_rx[0]==2 && flag_valve_doing_cyclic_motion==false) // actuate the currently selected valve
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        Z_commanded_target_position = (stepper_Z[active_valve_ID].currentPosition()+relative_position);
        stepper_Z[active_valve_ID].runToNewPosition(Z_commanded_target_position);
        // Z_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==3 && flag_valve_doing_cyclic_motion==false) // close the selected valve
      {
        active_valve_ID = buffer_rx[1];
        flag_cycling_selection_of_valve = false;
        flag_valve_doing_cyclic_motion = false;
        flag_close_valve_in_progress = true;
      }
      if(buffer_rx[0]==4) // valve cycling
      {
        // only allow valve cycling when all valves have been closed
        bool tmp = true;
        for(int i=0;i++;i<N_valves)
          tmp = tmp && flag_valve_close_position_reset[i];
        if(buffer_rx[1]==1 && tmp==true) // start cycling all the valves
          flag_valve_doing_cyclic_motion = true; 
        // stop cycling
        if(buffer_rx[1]==0) // stop cycling all the valves
          if(flag_valve_doing_cyclic_motion == true)
            flag_valve_stop_cyclic_motion_requested = true; 
      }
      if(buffer_rx[0]==5) // set the current active valve or enable valve_selection cycling (and set duration for each cycle)
      {
        if(buffer_rx[1]==0)
        {
          flag_cycling_selection_of_valve = false;
          active_valve_ID = buffer_rx[2];
        }
        else
        {
          flag_cycling_selection_of_valve = true;
          number_of_timer_cycles_per_valve = 256*int(buffer_rx[2]) + int(buffer_rx[3]);
        }        
      }
    }
  }

  if(flag_read_sensor)
  {
    enableMuxPort_SFM3000(active_valve_ID);
    ret_sfm3000 = sfm3000.read_sample();
    if (ret_sfm3000 == 0) 
      mFlow = sfm3000.get_flow();

    enableMuxPort_ABP(active_valve_ID);
    abp_30psi.update();
    mPressure = abp_30psi.pressure();
 
    flag_read_sensor = false;
  }

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
        SerialUSB.print(mPressure);
        SerialUSB.print("\n");
      }
      else
      {
        // field 1: time
        buffer_tx[buffer_tx_ptr++] = byte(cycle_count >> 24);
        buffer_tx[buffer_tx_ptr++] = byte(cycle_count >> 16);
        buffer_tx[buffer_tx_ptr++] = byte(cycle_count >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(cycle_count %256);

        // field 2: stepper pos
        tmp_uint16 = signed2NBytesUnsigned(stepper_Z[active_valve_ID].currentPosition(), 2);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

        // field 3: flow
        tmp_long = (65536 / 2) * mFlow / flow_FS;
        tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

        // field 4: upstream pressure
        tmp_uint16 = 65536 * mPressure / pressure_FS;
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

        // field 5: valve ID
        buffer_tx[buffer_tx_ptr++] = 0;
        buffer_tx[buffer_tx_ptr++] = active_valve_ID;

        // field 6: force
        buffer_tx[buffer_tx_ptr++] = byte(0 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(0 % 256);

        // field 7: aux (e.g additional steps needed to fully close the valve)
        buffer_tx[buffer_tx_ptr++] = byte(0 >> 8);
        buffer_tx[buffer_tx_ptr++] = byte(0 % 256);
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

  if (flag_close_valve_in_progress && ret_sfm3000 == 0)
  {
    if(mFlow >= 0.2)
    {
      flag_valve_flow_detected[active_valve_ID] = true;
      Z_commanded_target_position = (stepper_Z[active_valve_ID].currentPosition()+1);
      stepper_Z[active_valve_ID].runToNewPosition(Z_commanded_target_position);
    }
    else
    {
      if(flag_valve_flow_detected[active_valve_ID])
      {
        Z_commanded_target_position = (stepper_Z[active_valve_ID].currentPosition()+1);
        stepper_Z[active_valve_ID].runToNewPosition(Z_commanded_target_position);
        stepper_Z[active_valve_ID].setCurrentPosition(0);
        flag_valve_flow_detected[active_valve_ID] = false;
        flag_close_valve_in_progress = false;
        flag_valve_close_position_reset[active_valve_ID] = true;
      }
      else
      {
        flag_close_valve_in_progress = false;
        flag_valve_flow_detected[active_valve_ID] = false;
        flag_valve_close_position_reset[active_valve_ID] = false;
      }
    }
  }
  
  // run steppers
  for(int i = 0;i++;i<N_valves)
    stepper_Z[i].run();
  
}

/*******************************************************************
 ************************  TIMER CALLBACK  *************************
 *******************************************************************/
void timer_interruptHandler()
{
  // cycle the selection of the valve
  if(flag_cycling_selection_of_valve)
    if(++counter_valve_selection == number_of_timer_cycles_per_valve)
      active_valve_ID = (active_valve_ID+1)%N_valves;

  // set sensor reading flag
  flag_read_sensor = true;
  flag_write_data = true;

  // to have multiple readings for each linear actuator position
  timer_div_counter++;
  if(timer_div_counter>=timer_div)
  {
    timer_div_counter = 0;
    if(flag_valve_doing_cyclic_motion)
    {
       // cycle all the valves
      for(int i = 0; i++; i<N_valves)
      {
        if(stepper_Z[i].currentPosition() == 0)
        {
          // park at the fully closed position for LIMIT_WAIT_CYCLES timer cycles
          if (is_at_limit == false)
          {
            is_at_limit = true;
            limit_wait_counter = 0;
          }
          if (is_at_limit == true)
          {
            if (limit_wait_counter++ == LIMIT_WAIT_CYCLES)
            {
              cycle_count++;
              if(flag_valve_stop_cyclic_motion_requested==true)
              {
                flag_valve_doing_cyclic_motion = false;
                flag_valve_stop_cyclic_motion_requested = false;
              }
              else
              {
                is_at_limit = false;
                stepper_Z[i].moveTo(-cyclic_motion_limit_valve);
              }
            }
          }
        }
        if(stepper_Z[i].currentPosition() == -cyclic_motion_limit_valve)
        {
          // park at the fully open position for LIMIT_WAIT_CYCLES timer cycles
          if (is_at_limit == false)
          {
            is_at_limit = true;
            limit_wait_counter = 0;
          }
          if (is_at_limit == true)
          {
            if (limit_wait_counter++ == LIMIT_WAIT_CYCLES)
            {
              is_at_limit = false;
              stepper_Z[i].moveTo(0);
            }
          }
        }
      }
    }
  }
}

// utils
long signed2NBytesUnsigned(long signedLong,int N)
{
  long NBytesUnsigned = signedLong + pow(256L,N)/2;
  //long NBytesUnsigned = signedLong + 8388608L;
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

/***************************************************************************************************/
/*********************************************  utils  *********************************************/
/***************************************************************************************************/

//Enables a specific port number
boolean enableMuxPort_SFM3000(byte portNumber)
{
  byte settings = (1 << portNumber);
  Wire.beginTransmission(MUX_ADDR_SFM3000);
  Wire.write(settings);
  Wire.endTransmission();
  return(true);
}

boolean enableMuxPort_ABP(byte portNumber)
{
  byte settings = (1 << portNumber);
  Wire.beginTransmission(MUX_ADDR_ABP);
  Wire.write(settings);
  Wire.endTransmission();
  return(true);
}

static inline int sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
