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
TMC2209Stepper Y_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);

// driver 1 actuator 1
//static const int Y_dir = 28;
//static const int Y_step = 26;
//static const int Y_driver_uart = 25;
//static const int Y_en = 36;
//static const int Y_gnd = 37;
//static const long steps_per_mm_XY = 78.7*1; 
//constexpr float MAX_VELOCITY_Y_mm = 7; 
//constexpr float MAX_ACCELERATION_Y_mm = 100; // 50 ms to reach 15 mm/s
//static const long Y_NEG_LIMIT_MM = -12;
//static const long Y_POS_LIMIT_MM = 12;

// driver 2 - PL25
static const int Y_dir = 24;
static const int Y_step = 22;
static const int Y_driver_uart = 25;
static const int Y_en = 36;
static const int Y_gnd = 37;
static const long steps_per_mm_XY = 30*4; 
constexpr float MAX_VELOCITY_Y_mm = 25; 
constexpr float MAX_ACCELERATION_Y_mm = 500; // 50 ms to reach 15 mm/s
static const long Y_NEG_LIMIT_MM = -12;
static const long Y_POS_LIMIT_MM = 12;

AccelStepper stepper_Y = AccelStepper(AccelStepper::DRIVER, Y_step, Y_dir);
long Y_commanded_target_position = 0;
bool Y_commanded_movement_in_progress = false;

#include <DueTimer.h>
static const int TIMER_PERIOD = 500; // in us

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established
  buffer_rx_ptr = 0;

  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
    
  pinMode(Y_driver_uart, OUTPUT); // to remove
  pinMode(Y_dir, OUTPUT);
  pinMode(Y_step, OUTPUT);

  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);

  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);
  
  select_driver(2);
  while(!STEPPER_SERIAL);
  Y_driver.begin();
  Y_driver.I_scale_analog(false);  
  Y_driver.rms_current(200); //I_run and holdMultiplier
  Y_driver.microsteps(4);
  Y_driver.pwm_autoscale(true);
  Y_driver.TPOWERDOWN(2);
  Y_driver.en_spreadCycle(false);
  Y_driver.toff(4);

  stepper_Y.setEnablePin(Y_en);
  stepper_Y.setPinsInverted(false, false, true);
  stepper_Y.setMaxSpeed(MAX_VELOCITY_Y_mm*steps_per_mm_XY);
  stepper_Y.setAcceleration(MAX_ACCELERATION_Y_mm*steps_per_mm_XY);
  stepper_Y.enableOutputs();

  //ADC
  //ads1115.begin();
}

void loop() {

  // read one meesage from the buffer
  while (SerialUSB.available()) { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) {
      buffer_rx_ptr = 0;
      if(buffer_rx[0]==1)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        Y_commanded_target_position = (stepper_Y.currentPosition()+relative_position);
        stepper_Y.moveTo(Y_commanded_target_position);
        Y_commanded_movement_in_progress = true;
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
  if(Y_commanded_movement_in_progress && stepper_Y.currentPosition()==Y_commanded_target_position)
    Y_commanded_movement_in_progress = false;

  // move motors
  if(Y_commanded_movement_in_progress)
    stepper_Y.run();

  
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
}
