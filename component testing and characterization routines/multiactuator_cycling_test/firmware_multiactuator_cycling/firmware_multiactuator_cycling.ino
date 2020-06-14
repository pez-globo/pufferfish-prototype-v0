// SFM3x00 connection
//    Brown:    2 (RJ45)  SDA
//    Orange:   8 (RJ45)  GND
//    Blue:     4 (RJ45)  VCC (5V)
//    Green:    6 (RJ45)  SCK

/*************************************************************************************************************
 *  Multi-actuator cycling test-rig firmware. 
 *  Cycles multiple actuators at a constant frequency.
 *  Measures the true cycles by counting the transitions of a limit switch. 
 *  (optional) also measure temperatures of the actuators over time
 ************************************************************************************************************/
static const bool USE_SERIAL_MONITOR = false;
#include <Wire.h>

#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>
#include <DueTimer.h>
//----------------------------------------------------------------------------------------------

static inline int sgn(int val) 
{
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
//----------------------------------------------------------------------------------------------
static const float TIMER_PERIOD_us = 3000; // in us
static const int timer_div = 5;
volatile int timer_div_counter = 0;

long tmp_long;
uint16_t tmp_uint16;


const int N_valves = 4; // No:of valves/actuators being cycled simultaneously. 

//----------------------------------------------------------------------------------------------
// DIRECTION & STEP Pins
//----------------------------------------------------------------------------------------------
int DIRECTION[N_valves] {0,1,2,3};
int STEP[N_valves] = {4,5,6,7};

//----------------------------------------------------------------------------------------------
// LIMIT SWITCH Pins
//----------------------------------------------------------------------------------------------
int LIMIT_SWITCH[N_valves];

//----------------------------------------------------------------------------------------------
// State variables
//----------------------------------------------------------------------------------------------
bool flag_close_valve_in_progress[N_valves];

bool flag_valve_doing_cyclic_motion[N_valves];

volatile bool flag_valve_close_position_reset[N_valves]


int valve_cyclic_motion_dir[N_valves];

int valve_cyclic_motion_step_size[N_valves]; 


volatile bool flag_read_sensor = true;
volatile bool flag_write_data = true;


bool move_valve[N_valves] = {false};

bool cycle_valve[N_valves] = {false};

//----------------------------------------------------------------------------------------------
// State variables for limit finding and homing
//----------------------------------------------------------------------------------------------
bool home_valve[N_valves] = {false};
bool limitFinding_inProgress[N_valves] = {false};
bool limitFinding_complete[N_valves] = {false};
bool homing_inProgress[N_valves] = {false};
bool homing_complete[N_valves] = {false};
//----------------------------------------------------------------------------------------------


// byte[0]: which motor to move: 0 x, 1 y, 2 z, 3 LED, 4 Laser
// byte[1]: what direction: 1 forward, 0 backward
// byte[2]: how many micro steps - upper 8 bits
// byte[3]: how many micro steps - lower 8 bits

/*******************************************************************************
 ************************* SERIAL COMM VARIABLES *******************************
 *******************************************************************************/

static const int CMD_LENGTH = 4;
static const int MSG_LENGTH = 100;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

/*******************************************************************
 ************************* STEPPER CONTROL *************************
 *******************************************************************/
AccelStepper stepper[N_valves] = AccelStepper(AccelStepper::DRIVER, 0, 1);

for(int ii=0; ii<N_valves; ii++)
{
    stepper[ii] =  AccelStepper(AccelStepper::DRIVER, STEP[ii], DIRECTION[ii]);

}
 
static const int UART_CS_S0 = 46;
static const int UART_CS_S1 = 47;
#define STEPPER_SERIAL Serial3
static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;


// Create 1 instance of the TMC stepper object for setting parameters for all steppers. 
/// All stepper driver UARTS are tied to a common UART pin from the Arduino.

TMC2209Stepper Driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
  

//----------------------------------------------------------------------------------------------
// Motion Profile Variables
//----------------------------------------------------------------------------------------------

// driver 2 - PL25
//static const long steps_per_mm_XY = 30*4; 
//constexpr float MAX_VELOCITY_Y_mm = 25; 
//constexpr float MAX_ACCELERATION_Y_mm = 500; // 50 ms to reach 15 mm/s

static const int N_microstepping = 2;
static const long steps_per_mm = 78.74*Y_N_microstepping;  // NMB linear actuator 
//static const long steps_per_mm_Y = 19.68*Y_N_microstepping;    // Dings linear actuator (J)

constexpr float MAX_VELOCITY_mm = 7.62; 
constexpr float MAX_ACCELERATION_mm = 100;
static const long NEG_LIMIT_MM = -12;
static const long POS_LIMIT_MM = 12;

long cyclic_motion_amplitude = int(steps_per_mm*3);

long commanded_target_position[N_valves] = {0};
bool commanded_movement_in_progress[N_valves] = {false};

float phase[N_valves] = {0}

/*******************************************************************
 ************************* LIMIT FINDING VARIABLES  *************************
 *******************************************************************/
bool homing_in_progress[N_valves] = {false};
bool at_home[N_valves] = {false};
bool homing_complete[N_valves] = {false};

bool limit_finding_in_progress[N_valves] = {false}; 
bool limit_finding_complete[N_valves] = {false};


/*******************************************************************
 ************************* MEASURED VARIABLES  *************************
 *******************************************************************/
// Cycles
volatile long int n_cycles[N_valves] = {0};  // Tracks number of cycles for each actuator. 

// Temperature
float temp_reading[N_valves] = {0};  // Measured temperature for each actuator. 


void setup() 
{
  // wait for USB to connect
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established

  // reset rx buffer pointer
  buffer_rx_ptr = 0;

  // init stepper motors
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  /*******************************************************************
  // INITIALIZE STEP AND DIRECTIONS PINS AS OUTPUT
   *******************************************************************/
   // @@@ ADD CODE HERE @@@

  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);

  STEPPER_SERIAL.begin(115200);

  for(int ii=1;ii<=N_valves;ii++)
  {
    select_driver(ii);
    while(!STEPPER_SERIAL);
    Driver.begin();
    Driver.I_scale_analog(false);  
    Driver.rms_current(300,0.2); //I_run and holdMultiplier
    Driver.microsteps(X_N_microstepping);
    Driver.pwm_autoscale(true);
    Driver.TPOWERDOWN(2);
    Driver.en_spreadCycle(false);
    Driver.toff(4);

  }

   /*******************************************************************
    Set Motion Profile parameters for all AccelStepper instances
   *******************************************************************/
  // Convert to a FOR Loop
  for(int ii=0;ii<N_valves;ii++)
  {
    stepper[ii].setPinsInverted(false, false, true);
    stepper[ii].setMaxSpeed(MAX_VELOCITY_X_mm*steps_per_mm_X);
    stepper[ii].setAcceleration(MAX_ACCELERATION_X_mm*steps_per_mm_X);
    stepper[ii].enableOutputs();
  }


  /*******************************************************************
  Interrupts for reading the hall-effect sensor: 
   Will need to hardcode since interrupt handling functions cannot take inputs
   *******************************************************************/
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH[0]), HandleInterrupt_1, FALLING);

  // ...

  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH[N_valves]), HandleInterrupt_2, FALLING);


  



  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

  
}

void loop() 
{

  
  
  
  /* Read one message from the buffer
   *  Message format
   * - byte[0]: Actuator ID (0-255) if ID ==0  then apply command to all actuators.
       byte[1]: Direction/Type of movement command:
        - 0: Backward
        - 1: Forward
        - 2: Cycle actuators
        - 3: Home actuator
       byte[2]: Microsteps to move, upper 8 bits
       byte[3]: Microsteps to move, lower 8 bits
   */
  while (SerialUSB.available()) { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) 
    {
      buffer_rx_ptr = 0;

     
      int valve_id = buffer_rx[0];
    
      // Handle Move Commands
      if(buffer_rx[1] == 0 || buffer_rx[1] == 1)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        
        if(valve_id < N_valves)
        {
          move_valve[valve_id] = true;
          cycle_valve[valve_id] = false;
          home_valve[valve_id] = false;
          
          commanded_target_position = (stepper[valve_id].currentPosition() + relative_position);
          stepper[valve_id].runToNewPosition(commanded_target_position);
        }
        else
        {
          for(int ii=0; ii<N_valves;ii++)
          {
            move_valve[ii] = true;
            cycle_valve[ii] = false;
            home_valve[ii] = false;
            
            commanded_target_position = (stepper[ii].currentPosition() + relative_position);
            stepper[ii].runToNewPosition(commanded_target_position);
          }
        }
        
      }
      // Handle Cycle commands
      else if(buffer_rx[1] == 2)
      {
        if(valve_id < N_valves)
        {
          cycle_valve[valve_id] = true;
          home_valve[valve_id] = false;
        }
        else
        {
          for(int ii=0; ii<N_valves;ii++)
          {
            cycle_valve[ii] = true;
            home_valve[ii] = false;
          }
        }
        
      }
      // Handle homing command
      else if(buffer_rx[1] == 3)
      { 
        if(valve_id < N_valves)
        {
          home_valve[valve_id] = true;
          cycle_valve[valve_id] = false;
        }
        else
        {
          for(int ii=0; ii<N_valves;ii++)
          {
            home_valve[ii] = true;
            cycle_valve[ii] = false;
          }
        }
      }
      
      if(buffer_rx[0]==0 && flag_valve1_doing_cyclic_motion==false)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        X_commanded_target_position = (stepper_X.currentPosition()+relative_position);
        stepper_X.runToNewPosition(X_commanded_target_position);
        // X_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==1 && flag_valve2_doing_cyclic_motion==false)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        Y_commanded_target_position = (stepper_Y.currentPosition()+relative_position);
        stepper_Y.runToNewPosition(Y_commanded_target_position);
        // Y_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==2 && flag_valve3_doing_cyclic_motion==false)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        Z_commanded_target_position = (stepper_Z.currentPosition()+relative_position);
        stepper_Z.runToNewPosition(Z_commanded_target_position);
        // Z_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==3)
      {
        if(buffer_rx[1]==0)
        {
          flag_valve1_doing_cyclic_motion = false;
          flag_close_valve1_in_progress = true;
        }
        if(buffer_rx[1]==1)
        {
          flag_valve2_doing_cyclic_motion = false;
          flag_close_valve2_in_progress = true;
        }
        if(buffer_rx[1]==2)
        {
          flag_valve3_doing_cyclic_motion = false;
          flag_close_valve3_in_progress = true;
        }
      }
      if(buffer_rx[0]==4)
      {
        if(buffer_rx[1]==0 && flag_valve1_close_position_reset==true)
          flag_valve1_doing_cyclic_motion = true;
        if(buffer_rx[1]==1 && flag_valve2_close_position_reset==true)
          flag_valve2_doing_cyclic_motion = true;
          stepper_Y.setCurrentPosition(0);
        if(buffer_rx[1]==2 && flag_valve3_close_position_reset==true)
          flag_valve3_doing_cyclic_motion = true;
      }
    }
  }




  if(flag_write_data)
  {
        
      if(USE_SERIAL_MONITOR)
        {
          
        }
        else
        {
          /*Send data (N steppers)        
           * 1. Position (2 byte integer)
           * 2. Cycles (4 byte integet)
           * 3. Temperature (2 byte integer)
           */

          
          tmp_uint16 = signed2NBytesUnsigned(stepper_Y.currentPosition(), 2);
          buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
          buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

          // Measured force value
          tmp_uint16 = signed2NBytesUnsigned(force_value, 2);
          buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
          buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

        }
      

      
    // send data to computer
    if(buffer_tx_ptr==MSG_LENGTH)
    {
      SerialUSB.write(buffer_tx, MSG_LENGTH);
      buffer_tx_ptr = 0;
    }
    // clear the flag
    flag_write_data = false;
  }

  
    
  

  // run steppers
  stepper_X.run();
  stepper_Y.run();
  stepper_Z.run();
  
}

void timer_interruptHandler()
{
  flag_read_sensor = true;
  flag_write_data = true;

  // to have multiple readings for each linear actuator position
  timer_div_counter++;
  if(timer_div_counter>=timer_div)
  {
    timer_div_counter = 0;


    for(int ii=0; ii< N_valves; ii++)
    {

      if(home_valve[ii] == true)
      {
        
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


float readTempSensor(int sensor)
{
  float sensor_value = analogRead(sensor);

  return sensor_value;
}

void HandleInterrupt_1()
{
  // Update counter 1
  n_cycles[0]++;
}
void HandleInterrupt_2()
{
  // Update counter 2
  n_cycles[1]++;
}
void HandleInterrupt_3()
{
  // Update counter 3
  n_cycles[2]++;
}
void HandleInterrupt_4()
{
  // Update counter 4
  n_cycles[3]++;
}
void HandleInterrupt_5()
{
  // Update counter 5
  n_cycles[4]++;
}
void HandleInterrupt_6()
{
  // Update counter 6
  n_cycles[5]++;
}
void HandleInterrupt_7()
{
  // Update counter 7
  n_cycles[6]++;
}
void HandleInterrupt_8()
{
  // Update counter 8
  n_cycles[7]++;
}
void HandleInterrupt_9()
{
  // Update counter 9
  n_cycles[8]++;
}
void HandleInterrupt_10()
{
  // Update counter 10
  n_cycles[9]++;
}
