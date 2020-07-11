#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>

static inline int sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

// byte[0]: which motor to move: 0 x, 1 y, 2 z, 3 LED, 4 Laser
// byte[1]: what direction: 1 forward, 0 backward
// byte[2]: how many micro steps - upper 8 bits
// byte[3]: how many micro steps - lower 8 bits

static const int CMD_LENGTH = 4;
static const int MSG_LENGTH = 9;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
static const int N_BYTES_POS = 3;

// stepper
static const int UART_CS_S0 = 46;
static const int UART_CS_S1 = 47;
#define STEPPER_SERIAL Serial3
static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;
TMC2209Stepper X_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
TMC2209Stepper Y_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
TMC2209Stepper Z_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);

// driver 1 actuator 1
static const int X_dir = 28;
static const int X_step = 26;
static const int X_en = 36;
static const int X_N_microstepping = 2;
static const long steps_per_mm_X = 78.74*X_N_microstepping; 
constexpr float MAX_VELOCITY_X_mm = 7.62; 
constexpr float MAX_ACCELERATION_X_mm = 100;
static const long X_NEG_LIMIT_MM = -12;
static const long X_POS_LIMIT_MM = 12;

// driver 2 - PL25
//static const long steps_per_mm_XY = 30*4; 
//constexpr float MAX_VELOCITY_Y_mm = 25; 
//constexpr float MAX_ACCELERATION_Y_mm = 500; // 50 ms to reach 15 mm/s
static const int Y_dir = 24;
static const int Y_step = 22;
static const int Y_en = 36;
static const int Y_N_microstepping = 2;
static const long steps_per_mm_Y = 78.74*Y_N_microstepping; 
constexpr float MAX_VELOCITY_Y_mm = 7.62; 
constexpr float MAX_ACCELERATION_Y_mm = 100;
static const long Y_NEG_LIMIT_MM = -12;
static const long Y_POS_LIMIT_MM = 12;

static const int Z_dir = 23;
static const int Z_step = 25;
static const int Z_N_microstepping = 2;
static const long steps_per_mm_Z = 82.02*Y_N_microstepping; 
constexpr float MAX_VELOCITY_Z_mm = 18.29; 
constexpr float MAX_ACCELERATION_Z_mm = 100;
static const long Z_NEG_LIMIT_MM = -12;
static const long Z_POS_LIMIT_MM = 12;

AccelStepper stepper_X = AccelStepper(AccelStepper::DRIVER, X_step, X_dir);
AccelStepper stepper_Y = AccelStepper(AccelStepper::DRIVER, Y_step, Y_dir);
AccelStepper stepper_Z = AccelStepper(AccelStepper::DRIVER, Z_step, Z_dir);

long X_commanded_target_position = 0;
bool X_commanded_movement_in_progress = false;
long Y_commanded_target_position = 0;
bool Y_commanded_movement_in_progress = false;
long Z_commanded_target_position = 0;
bool Z_commanded_movement_in_progress = false;

#include <DueTimer.h>
static const int TIMER_PERIOD = 500; // in us

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established
  buffer_rx_ptr = 0;

  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
    
  pinMode(X_dir, OUTPUT);
  pinMode(X_step, OUTPUT);

  pinMode(Y_dir, OUTPUT);
  pinMode(Y_step, OUTPUT);

  pinMode(Z_dir, OUTPUT);
  pinMode(Z_step, OUTPUT);

  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);

  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);

  select_driver(1);
  while(!STEPPER_SERIAL);
  X_driver.begin();
  X_driver.I_scale_analog(false);  
  X_driver.rms_current(300,0.2); //I_run and holdMultiplier
  X_driver.microsteps(X_N_microstepping);
  X_driver.pwm_autoscale(true);
  X_driver.TPOWERDOWN(2);
  X_driver.en_spreadCycle(false);
  X_driver.toff(4);
  stepper_X.setPinsInverted(false, false, true);
  stepper_X.setMaxSpeed(MAX_VELOCITY_X_mm*steps_per_mm_X);
  stepper_X.setAcceleration(MAX_ACCELERATION_X_mm*steps_per_mm_X);
  stepper_X.enableOutputs();
  
  select_driver(2);
  while(!STEPPER_SERIAL);
  Y_driver.begin();
  Y_driver.I_scale_analog(false);  
  Y_driver.rms_current(300,0.2); //I_run and holdMultiplier
  Y_driver.microsteps(Y_N_microstepping);
  Y_driver.pwm_autoscale(true);
  Y_driver.TPOWERDOWN(2);
  Y_driver.en_spreadCycle(false);
  Y_driver.toff(4);
  stepper_Y.setPinsInverted(false, false, true);
  stepper_Y.setMaxSpeed(MAX_VELOCITY_Y_mm*steps_per_mm_Y);
  stepper_Y.setAcceleration(MAX_ACCELERATION_Y_mm*steps_per_mm_Y);
  stepper_Y.enableOutputs();

  select_driver(3);
  while(!STEPPER_SERIAL);
  Z_driver.begin();
  Z_driver.I_scale_analog(false);  
  Z_driver.rms_current(500,0.2); //I_run and holdMultiplier
  Z_driver.microsteps(Z_N_microstepping);
  Z_driver.pwm_autoscale(true);
  Z_driver.TPOWERDOWN(2);
  Z_driver.en_spreadCycle(false);
  Z_driver.toff(4);
  stepper_Z.setPinsInverted(false, false, true);
  stepper_Z.setMaxSpeed(MAX_VELOCITY_Z_mm*steps_per_mm_Z);
  stepper_Z.setAcceleration(MAX_ACCELERATION_Z_mm*steps_per_mm_Z);
  stepper_Z.enableOutputs();
  
}

void loop() {

  // read one meesage from the buffer
  while (SerialUSB.available()) { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) {
      buffer_rx_ptr = 0;
      if(buffer_rx[0]==0)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        X_commanded_target_position = (stepper_X.currentPosition()+relative_position);
        stepper_X.moveTo(X_commanded_target_position);
        X_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==1)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        Y_commanded_target_position = (stepper_Y.currentPosition()+relative_position);
        stepper_Y.moveTo(Y_commanded_target_position);
        Y_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==2)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        Z_commanded_target_position = (stepper_Z.currentPosition()+relative_position);
        stepper_Z.moveTo(Z_commanded_target_position);
        Z_commanded_movement_in_progress = true;
      }
      
      /*
      SerialUSB.print(buffer_rx[0]);
      SerialUSB.print(buffer_rx[1]);
      SerialUSB.print(buffer_rx[2]);
      SerialUSB.print(buffer_rx[3]);
      SerialUSB.print('#');
      */
      
      //break; // exit the while loop after reading one message
    }
  }


  // check if commanded position has been reached
  if(X_commanded_movement_in_progress && stepper_X.currentPosition()==X_commanded_target_position)
    X_commanded_movement_in_progress = false;
  // move motors
  if(X_commanded_movement_in_progress)
    stepper_X.run();
  
  if(Y_commanded_movement_in_progress && stepper_Y.currentPosition()==Y_commanded_target_position)
    Y_commanded_movement_in_progress = false;
  // move motors
  if(Y_commanded_movement_in_progress)
    stepper_Y.run();

  if(Z_commanded_movement_in_progress && stepper_Z.currentPosition()==Z_commanded_target_position)
    Z_commanded_movement_in_progress = false;
  // move motors
  if(Z_commanded_movement_in_progress)
    stepper_Z.run();

  
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
