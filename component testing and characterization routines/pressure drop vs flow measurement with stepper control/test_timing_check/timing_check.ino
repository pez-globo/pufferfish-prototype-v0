// SFM3x00 connection
//    Brown:    2 (RJ45)  SDA
//    Orange:   8 (RJ45)  GND
//    Blue:     4 (RJ45)  VCC (5V)
//    Green:    6 (RJ45)  SCK

static const bool USE_SERIAL_MONITOR = true;

#include <Wire.h>
#include <sfm3x00.h>
#include <HoneywellTruStabilitySPI.h>
#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>
#include <DueTimer.h>

static const float TIMER_PERIOD_us = 5000; // in us

long tmp_long;
uint16_t tmp_uint16;
float flow_FS = 200;
float pressure_FS = 60;

bool flag_close_valve1_in_progress = false;
bool flag_close_valve2_in_progress = false;
bool flag_close_valve3_in_progress = false;
float flow_th = 0.2;

volatile bool flag_read_sensor = true;
volatile bool flag_write_data = true;

volatile bool flag_valve1_flow_detected = false;
volatile bool flag_valve2_flow_detected = false;
volatile bool flag_valve3_flow_detected = false;

volatile bool flag_valve1_close_position_reset = false;
volatile bool flag_valve2_close_position_reset = false;
volatile bool flag_valve3_close_position_reset = false;

volatile bool flag_valve1_doing_cyclic_motion = false;
volatile bool flag_valve2_doing_cyclic_motion = false;
volatile bool flag_valve3_doing_cyclic_motion = false;

int valve1_cyclic_motion_dir = 1;
int valve2_cyclic_motion_dir = 1;
int valve3_cyclic_motion_dir = 1;

int valve1_cyclic_motion_step_size = 1;
int valve2_cyclic_motion_step_size = 1;
int valve3_cyclic_motion_step_size = 1;

long cyclic_motion_limit_valve1 = 500;
long cyclic_motion_limit_valve2 = 500;
long cyclic_motion_limit_valve3 = 500;

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
static const int MSG_LENGTH = 504;
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
TMC2209Stepper X_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
TMC2209Stepper Y_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
TMC2209Stepper Z_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);

// driver 1 actuator 1
static const int X_dir = 28;
static const int X_step = 26;
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

// pin define
#define SLAVE_SELECT_PIN_1 30
#define SLAVE_SELECT_PIN_2 31
#define SLAVE_SELECT_PIN_3 32
#define SLAVE_SELECT_PIN_4 33

/*******************************************************************
 ***************************** SENSORS *****************************
 *******************************************************************/

SFM3000 sfm3000;

//HSCDANN060PGSA3
TruStabilityPressureSensor hsc_sensor_1( SLAVE_SELECT_PIN_1, 0, 60 ); // HSCDANN005PGSA3
TruStabilityPressureSensor hsc_sensor_2( SLAVE_SELECT_PIN_2, 0, 60 ); // HSCDANN005PGSA3
TruStabilityPressureSensor hsc_sensor_3( SLAVE_SELECT_PIN_3, 0, 60 ); // HSCDANN005PGSA3
TruStabilityPressureSensor hsc_sensor_4( SLAVE_SELECT_PIN_4, 0, 60 ); // HSCDANN005PGSA3

float mFlow;
float mPressure_1;
float mPressure_2;
float mPressure_3;

int ret_sfm3000;
int ret_hsc_sensor_1;
int ret_hsc_sensor_2;
int ret_hsc_sensor_3;

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
    
  pinMode(X_dir, OUTPUT);
  pinMode(X_step, OUTPUT);

  pinMode(Y_dir, OUTPUT);
  pinMode(Y_step, OUTPUT);

  pinMode(Z_dir, OUTPUT);
  pinMode(Z_step, OUTPUT);

  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);

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

  // init the HSC sensor
  SPI.begin(); // start SPI communication
  hsc_sensor_1.begin(); // run sensor initialization
  hsc_sensor_2.begin(); // run sensor initialization
  hsc_sensor_3.begin(); // run sensor initialization

}

void loop() 
{
  flag_read_sensor = true;
  if(flag_read_sensor)
  {
    for(int i = 0;i<1000;i++)
    {
      ret_sfm3000 = sfm3000.read_sample();
      ret_hsc_sensor_1 = hsc_sensor_1.readSensor();
      ret_hsc_sensor_2 = hsc_sensor_2.readSensor();
      ret_hsc_sensor_3 = hsc_sensor_3.readSensor();
      // ret_hsc_sensor_4 = hsc_sensor_1.readSensor();
      
      if (ret_sfm3000 == 0) 
        mFlow = sfm3000.get_flow();
      
      if (ret_hsc_sensor_1+ret_hsc_sensor_2+ret_hsc_sensor_3 == 0)
      {
        mPressure_1 = hsc_sensor_1.pressure();
       mPressure_2 = hsc_sensor_2.pressure();
       mPressure_3 = hsc_sensor_3.pressure();
      }
    }
    flag_read_sensor = false;
  }

  flag_write_data = true;
  if(flag_write_data)
  {
    // only write data if the last read is successful
    if (ret_sfm3000 + ret_hsc_sensor_1 + ret_hsc_sensor_2 + ret_hsc_sensor_3 == 0)
    {
      if(USE_SERIAL_MONITOR)
      {
        // SerialUSB.print("flow rate (slm): ");
        SerialUSB.print(mFlow);
        SerialUSB.print(",");
        //SerialUSB.print(" pressure (cmH2O): ");
        SerialUSB.print(mPressure_1);
        SerialUSB.print(",");
        SerialUSB.print(mPressure_2);
        SerialUSB.print(",");
        SerialUSB.print(mPressure_3);
        SerialUSB.print("\n");
      }
    }
    flag_write_data = false;
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
