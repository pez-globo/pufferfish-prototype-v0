#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>

// v0.1.1 pin def

static const int Y_dir = 34;
static const int Y_step = 35;
static const int Y_driver_uart = 25;
static const int Y_en = 36;
static const int Y_gnd = 37;

#define STEPPER_SERIAL Serial1 
static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;
TMC2209Stepper Y_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);

#include <AccelStepper.h>
AccelStepper stepper_Y = AccelStepper(AccelStepper::DRIVER, Y_step, Y_dir);
static const long steps_per_mm_XY = 100; // for PL35L-024-VLB8
constexpr float MAX_VELOCITY_Y_mm = 20; // for PL35L-024-VLB8
//constexpr float MAX_VELOCITY_Y_mm = 1; // for PL35L-024-VLB8
constexpr float MAX_ACCELERATION_Y_mm = 200; // 50 ms to reach 15 mm/s
static const long Y_NEG_LIMIT_MM = -12;
static const long Y_POS_LIMIT_MM = 12;

void setup() {
  
  pinMode(Y_driver_uart, OUTPUT);
  pinMode(Y_dir, OUTPUT);
  pinMode(Y_step, OUTPUT);
  pinMode(Y_gnd, OUTPUT);
  digitalWrite(Y_gnd, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
   

  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);
  
  digitalWrite(Y_driver_uart, true);
  while(!STEPPER_SERIAL);
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
  stepper_Y.setMaxSpeed(MAX_VELOCITY_Y_mm*steps_per_mm_XY);
  stepper_Y.setAcceleration(MAX_ACCELERATION_Y_mm*steps_per_mm_XY);
  stepper_Y.enableOutputs();

}

void loop() {

  stepper_Y.runToNewPosition(-4*steps_per_mm_XY);
  stepper_Y.runToNewPosition(0);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  
}
