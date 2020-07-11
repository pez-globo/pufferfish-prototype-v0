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
static const bool USE_SERIAL_MONITOR = true;
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


const int N_valves = 2; // No:of valves/actuators being cycled simultaneously. 

const int MICROSTEPPING_N = 2;

long int test_counter[N_valves] = {0};
//----------------------------------------------------------------------------------------------
// DIRECTION & STEP Pins
//----------------------------------------------------------------------------------------------
//int DIRECTION[N_valves] {28, 24, 23, 27};
//int STEP[N_valves] = {26, 22, 25, 29};
int DIRECTION[N_valves] = {28, 24};
int STEP[N_valves] = {26, 22};

// Steppers
AccelStepper stepper[N_valves] = {AccelStepper(AccelStepper::DRIVER, 26 , 28), AccelStepper(AccelStepper::DRIVER, 22, 24)};

//----------------------------------------------------------------------------------------------
// LIMIT SWITCH Pins
//----------------------------------------------------------------------------------------------
int LIMIT_SWITCH[N_valves];

//----------------------------------------------------------------------------------------------
// State variables
//----------------------------------------------------------------------------------------------
bool flag_close_valve_in_progress[N_valves];

volatile bool flag_valve_close_position_reset[N_valves];

volatile bool flag_read_sensor = true;
volatile bool flag_write_data = true;


bool move_valve[N_valves] = {false};

bool cycle_valve[N_valves] = {false};

volatile bool is_at_limit[N_valves] = {false};

static const int LIMIT_WAIT_CYCLES = 10;
volatile int limit_wait_counter[N_valves] = {0};

// Limit switch state variables
volatile bool lim_switch_reading[N_valves] = {HIGH};
volatile bool lim_switch_reading_prev[N_valves] = {HIGH};
volatile bool lim_switch_state[N_valves] = {HIGH}; 
volatile bool lim_switch_state_prev[N_valves] = {HIGH}; // State of the limit switch: false when closed, true when open

//----------------------------------------------------------------------------------------------
// State variables for limit finding and homing
//----------------------------------------------------------------------------------------------
bool home_valve[N_valves] = {false};
bool limitFinding_inProgress[N_valves] = {false};
bool limitFinding_complete[N_valves] = {false};
bool homing_inProgress[N_valves] = {false};
bool homing_complete[N_valves] = {false};

unsigned long last_debounce_time[N_valves] = {0};  // the last time the output pin was toggled
unsigned long debounce_delay = 50;    // the debounce time; increase if the output flickers
//----------------------------------------------------------------------------------------------


// byte[0]: which motor to move: 0 x, 1 y, 2 z, 3 LED, 4 Laser
// byte[1]: what direction: 1 forward, 0 backward
// byte[2]: how many micro steps - upper 8 bits
// byte[3]: how many micro steps - lower 8 bits

/*******************************************************************************
 ************************* SERIAL COMM VARIABLES *******************************
 *******************************************************************************/

static const int CMD_LENGTH = 4;
static const int BYTES_PER_RECORD = 8;
static const int MESSAGE_BUNDLE_SIZE = 10;
static const int MSG_LENGTH = N_valves*BYTES_PER_RECORD*MESSAGE_BUNDLE_SIZE;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

/*******************************************************************
 ************************* STEPPER CONTROL *************************
 *******************************************************************/

 
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
static const long steps_per_mm = 78.74*MICROSTEPPING_N;  // NMB linear actuator 
//static const long steps_per_mm_Y = 19.68*Y_N_microstepping;    // Dings linear actuator (J)

constexpr float MAX_VELOCITY_mm = 7.62; 
constexpr float MAX_ACCELERATION_mm = 100;
static const long NEG_LIMIT_MM = -12;
static const long POS_LIMIT_MM = 12;

long cyclic_motion_amplitude = int(steps_per_mm*3);

long commanded_target_position[N_valves] = {0};
bool commanded_movement_in_progress[N_valves] = {false};

int valve_direction[N_valves] = {1};

long int valve_initial_position[N_valves] = {0};

float phase[N_valves] = {0};

int valve_cyclic_motion_dir[N_valves];
int valve_cyclic_motion_step_size[N_valves]; 

int limit_finding_step_size = 10;

long cyclic_motion_limit = 250*MICROSTEPPING_N;


/*******************************************************************
 ************************* MEASURED VARIABLES  *************************
 *******************************************************************/
// Cycles
volatile long int cycle_count[N_valves] = {0};  // Tracks number of cycles for each actuator. 

// Temperature
float temperature_reading[N_valves] = {0};  // Measured temperature for each actuator. 

int sensor_cycles = 0;
const int READ_SENSOR_CYCLES = 100;     // No:of timer cycles between analog sensor (temperature + force) reads.

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
  for(int ii=1;ii<=N_valves;ii++)
  {
    pinMode(STEP[ii], OUTPUT); pinMode(DIRECTION[ii], OUTPUT);
    pinMode(LIMIT_SWITCH[ii], INPUT_PULLUP);
    // Initialize AccelStepper objects
//    stepper[ii] =  AccelStepper(AccelStepper::DRIVER, STEP[ii], DIRECTION[ii]);
  }

  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);

  STEPPER_SERIAL.begin(115200);

  for(int ii=1;ii<=N_valves;ii++)
  { 
   
    select_driver(ii);
    while(!STEPPER_SERIAL);
    Driver.begin();
    Driver.I_scale_analog(false);  
    Driver.rms_current(100,0.3); //I_run and holdMultiplier
    Driver.microsteps(MICROSTEPPING_N);
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
    stepper[ii].setMaxSpeed(MAX_VELOCITY_mm*steps_per_mm);
    stepper[ii].setAcceleration(MAX_ACCELERATION_mm*steps_per_mm);
    stepper[ii].enableOutputs();

    // Set valve initial positions
  }


  /*******************************************************************
  Interrupts for reading the hall-effect sensor (only for counting cycles): 
   Will need to hardcode since interrupt handling functions cannot take inputs (!)
   *******************************************************************/
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH[0]), HandleInterrupt_1, FALLING);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH[1]), HandleInterrupt_2, FALLING);
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH[2]), HandleInterrupt_3, FALLING);
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH[3]), HandleInterrupt_4, FALLING);


  



  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

  
}

void loop() 
{
  /* Read one message from the buffer and parse it.
   *  Message format
   * - byte[0]: Actuator ID (0-255) if ID > N_valves  then apply command to all actuators.
       byte[1]: Direction/Type of movement command:
        - 0: Backward
        - 1: Forward
        - 2: Start Cycling Actuators
        - 3: Stop Cycling Actuators
        - 4: Home actuator
       byte[2]: Microsteps to move, upper 8 bits
       byte[3]: Microsteps to move, lower 8 bits
   */
  while (SerialUSB.available()) 
  { 
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
         
          cycle_valve[valve_id] = false;
          home_valve[valve_id] = false;
          
          commanded_target_position[valve_id] = (stepper[valve_id].currentPosition() + relative_position);
          stepper[valve_id].runToNewPosition(commanded_target_position[valve_id]);
        }
        else
        {
          for(int ii=0; ii<N_valves;ii++)
          {
            
            cycle_valve[ii] = false;
            home_valve[ii] = false;
            
            commanded_target_position[ii] = (stepper[ii].currentPosition() + relative_position);
            stepper[ii].runToNewPosition(commanded_target_position[ii]);
          }
        }
        
      }
      // Handle Start Cycle commands
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
      // Stop cycling valves
      else if(buffer_rx[1] == 3)
      {
        if(valve_id < N_valves)
        {
          cycle_valve[valve_id] = false;
         
        }
        else
        {
          for(int ii=0; ii<N_valves;ii++)
          {
            cycle_valve[ii] = false;
          
          }
        }
        
      }
      // Handle homing command
      else if(buffer_rx[1] == 4)
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
      
      
    }
  }


  
  // Send data to the computer
  if(flag_write_data)
  {
        
//        if(USE_SERIAL_MONITOR)
//        {
//            SerialUSB.println(buffer_tx_ptr);
//     
//        }
//        else
//        {
          /*Send data (N steppers)        
           * 1. Position (2 byte integer)
           * 2. Cycles (4 byte integet)
           * 3. Temperature (2 byte integer)
           */

     
          for (int valve_id=0; valve_id < N_valves; valve_id++)
          { 
             // Send the no:of cycles (4 byte unsigned int)
            long int temp = cycle_count[valve_id];
            // Testing
//            long int temp = test_counter[valve_id];
            buffer_tx[buffer_tx_ptr++] = byte(temp >>24);
            buffer_tx[buffer_tx_ptr++] = byte(temp >>16);
            buffer_tx[buffer_tx_ptr++] = byte(temp >> 8);
            buffer_tx[buffer_tx_ptr++] = byte(temp % 256);
            
            // Send the actuator positions (2 byte signed int)
            tmp_uint16 = signed2NBytesUnsigned(stepper[valve_id].currentPosition(), 2);

            // Testing
//            tmp_uint16 = signed2NBytesUnsigned(20 + valve_id, 2);
            buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 >> 8);
            buffer_tx[buffer_tx_ptr++] = byte(tmp_uint16 % 256);

            // Send the measured temperatures
//            int temperature_value = temperature_reading[valve_id];
            // Testing
            int temperature_value = 2*(valve_id+25);

//            int temperature_value = move_valve[valve_id];
            buffer_tx[buffer_tx_ptr++] = byte(temperature_value >> 8);
            buffer_tx[buffer_tx_ptr++] = byte(temperature_value % 256);

            
            
            
          }
        

//        }
      
      // send data to computer
      if(buffer_tx_ptr==MSG_LENGTH)
      {
        SerialUSB.write(buffer_tx, MSG_LENGTH);
        buffer_tx_ptr = 0;
        
      }
      // clear the flag
      flag_write_data = false;
  }

  // Handle homing commands

  for(int ii=0; ii< N_valves; ii++)
  {

     if(home_valve[ii])
     {
        // Read limit switch corresponding to valve ii
        ReadLimitSwitchDigital(ii);
        
        // Set the limit finding state
        if(limitFinding_complete[ii] == false && limitFinding_inProgress[ii] == false)
        {
          limitFinding_inProgress[ii] = true;
        }
        else if(limitFinding_complete[ii] == true && homing_inProgress[ii] == false)
        {
          homing_inProgress[ii] = true;
        }
     

        
      
        if(limitFinding_inProgress[ii] == true && limitFinding_complete[ii] == false)
        {
          FindLimits(ii);
          
        }
        else if(limitFinding_complete[ii] == true && homing_complete[ii] == false)
        {     
          // run homing
          // Move till you fully close the valve (home position)
          MoveToHome(ii);
        }
        
     }
     else
     {
        continue;
     }
  }
  
    
  

  // run steppers
  for (int valve_id = 0; valve_id < N_valves; valve_id++)
  {
    stepper[valve_id].run();
  }
  
}

void timer_interruptHandler()
{ 
  if(sensor_cycles >= READ_SENSOR_CYCLES)
  {
    flag_read_sensor = true;
    sensor_cycles = 0;
  }
  else
  {
    sensor_cycles++;
    
  }
  
  flag_write_data = true;

  // to have multiple readings for each linear actuator position
  timer_div_counter++;
  if(timer_div_counter>=timer_div)
  {
    timer_div_counter = 0;


    for(int valve_id = 0; valve_id < N_valves; valve_id++)
    {

      if(cycle_valve[valve_id] == true)
      {
        test_counter[valve_id]++;
        
        if(stepper[valve_id].currentPosition() == 0)
        {
          // park at the fully closed position for LIMIT_WAIT_CYCLES timer cycles
          if (is_at_limit[valve_id] == false)
          {
            is_at_limit[valve_id] = true;
            limit_wait_counter[valve_id] = 0;
          }
          if (is_at_limit[valve_id] == true)
          {
            if (limit_wait_counter[valve_id]++ == LIMIT_WAIT_CYCLES)
            {
              cycle_count[valve_id]++;
              is_at_limit[valve_id] = false;
              stepper[valve_id].moveTo(-cyclic_motion_limit);
            }
          }
        }
        if(stepper[valve_id].currentPosition() == -cyclic_motion_limit)
        {
          // park at the fully open position for LIMIT_WAIT_CYCLES timer cycles
          if (is_at_limit[valve_id] == false)
          {
            is_at_limit[valve_id] = true;
            limit_wait_counter[valve_id] = 0;
          }
          if (is_at_limit[valve_id] == true)
          {
            if (limit_wait_counter[valve_id]++ == LIMIT_WAIT_CYCLES)
            {
              is_at_limit[valve_id] = false;
              stepper[valve_id].moveTo(0);
            }
          }
        }

        
      }

      
    }
    
 
  }
}

void initialize_homing_state(int valve_id)
{
 
  
  limitFinding_inProgress[valve_id] = false;
  limitFinding_complete[valve_id] = false;

  homing_inProgress[valve_id] = false;
  homing_complete[valve_id] = false;
  home_valve[valve_id] = false;
  
  
}

void FindLimits(int valve_id)
{
 
  // If switch transitions from Open to Closed when stepping in OPEN direction. 
  if(lim_switch_state[valve_id] == LOW && lim_switch_state_prev[valve_id] == HIGH && limitFinding_complete[valve_id] == false)
  {   
//      limit_switch_finding_time[valve_id] = millis() - limit_switch_finding_start_time[valve_id];  

      limitFinding_complete[valve_id] = true;
      limitFinding_inProgress[valve_id] = false;
   
      valve_direction[valve_id] = -1;   
  }
  

  stepper[valve_id].move(valve_direction[valve_id]*limit_finding_step_size*MICROSTEPPING_N);

  
  
}

void MoveToHome(int valve_id)
{ 
  
  if(homing_inProgress[valve_id] == false)
  { 
    
    // Change the direction CLOSE
    valve_direction[valve_id] = -1;
    // Set the current position as 0
    stepper[valve_id].setCurrentPosition(0);
    stepper[valve_id].move(valve_direction[valve_id]*steps_per_mm*MICROSTEPPING_N*valve_initial_position[valve_id]);
    homing_inProgress[valve_id] = true;
  
  }

  if(homing_inProgress[valve_id] == true && stepper[valve_id].distanceToGo()==0)
  {

    homing_complete[valve_id] = true;
    
    initialize_homing_state(valve_id);

    stepper[valve_id].setCurrentPosition(0);
    
  }
}

void ReadLimitSwitchDigital(int valve_id)
{
  
  lim_switch_reading[valve_id] = digitalRead(LIMIT_SWITCH[valve_id]);

  if(lim_switch_reading[valve_id] != lim_switch_reading_prev[valve_id])
  {
    // State of the switch has changed either due to a real change or a noise.
    last_debounce_time[valve_id] = millis();
  }
  if(millis() - last_debounce_time[valve_id] > debounce_delay)
  {
    // The switch has stayed in the new state for at least debouce_delay
    // Therefore take it as the new limit switch state
      lim_switch_state_prev[valve_id] =  lim_switch_state[valve_id];
      lim_switch_state[valve_id] = lim_switch_reading[valve_id];
      
  }

  lim_switch_reading_prev[valve_id] = lim_switch_reading[valve_id];
  
  
}



float readTempSensor(int sensor)
{
  float sensor_value = analogRead(sensor);

  return sensor_value;
}

void HandleInterrupt_1()
{
  // Update counter 1
  cycle_count[0]++;
}
void HandleInterrupt_2()
{
  // Update counter 2
  cycle_count[1]++;
}
void HandleInterrupt_3()
{
  // Update counter 3
  cycle_count[2]++;
}
void HandleInterrupt_4()
{
  // Update counter 4
  cycle_count[3]++;
}
void HandleInterrupt_5()
{
  // Update counter 5
  cycle_count[4]++;
}
void HandleInterrupt_6()
{
  // Update counter 6
  cycle_count[5]++;
}
void HandleInterrupt_7()
{
  // Update counter 7
  cycle_count[6]++;
}
void HandleInterrupt_8()
{
  // Update counter 8
  cycle_count[7]++;
}
void HandleInterrupt_9()
{
  // Update counter 9
  cycle_count[8]++;
}
void HandleInterrupt_10()
{
  // Update counter 10
  cycle_count[9]++;
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
