#include <Wire.h>
#include <sfm3x00.h>
#include <sdpsensor-fast.h>
#include "Honeywell_ABP.h"
#include "pwm_lib.h"
#define MUX_ADDR 0x70 //7-bit unshifted default I2C Address

/***************************************************************************************************/
/********************************************* pwm_lib *********************************************/
/***************************************************************************************************/

using namespace arduino_due::pwm_lib;

//#define PWM_FREQUENCY 10000
//#define PWM_PERIOD_PIN_35 1000 // 10000 x (1e-8 secs) = 1e-4 sec
//#define PWM_DUTY_PIN_35 500

#define PWM_FREQUENCY 10000
#define PWM_PERIOD_PIN_35 10000 // 10000 x (1e-8 secs) = 1e-4 sec
#define PWM_DUTY_PIN_35 5000

// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWMH0_PC3> pwm_pin35;
pwm_wrapper<
  decltype(pwm_pin35)
> pwm_wrapper_pin35(pwm_pin35);
pwm_base* pwm_wrapper_pin35_ptr=&pwm_wrapper_pin35;

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
Honeywell_ABP abp_5psi_1(
  0x28,   // I2C address
  0,      // minimum pressure
  5,      // maximum pressure
  "psi"   // pressure unit
);
Honeywell_ABP abp_5psi_2(
  0x28,   // I2C address
  0,      // minimum pressure
  5,      // maximum pressure
  "psi"   // pressure unit
);
Honeywell_ABP abp_5psi_3(
  0x28,   // I2C address
  0,      // minimum pressure
  5,      // maximum pressure
  "psi"   // pressure unit
);

uint8_t cmd[1];
int ret_sfm3000_1 = 0;
int ret_sfm3000_2 = 0;
float dP;

void setup() 
{

  //extra ground
  pinMode(18,OUTPUT);
  digitalWrite(18,LOW);
  
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established

  Wire.setClock(400000);
  Wire.begin();

  // initialize the SFM sensor 1
  enableMuxPort(0);
  while(true) 
  {
    int ret_sfm3000_1 = sfm3000_1.init();
    if (ret_sfm3000_1 == 0) 
    {
      SerialUSB.print("init() for sensor 1: success\n");
      break;
    } 
    else 
    {
      SerialUSB.print("init() for sensor 1: failed, ret = ");
      SerialUSB.println(ret_sfm3000_1);
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
      SerialUSB.print("init() for sensor 2: success\n");
      break;
    } 
    else 
    {
        SerialUSB.print("init() for sensor 2: failed, ret = ");
        SerialUSB.println(ret_sfm3000_2);
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
      SerialUSB.print("init() for SDP810: success\n");
      break;
    }
    else
    {
      SerialUSB.print("init() for SDP810: failed, ret = ");
      SerialUSB.println(ret);
      delay(1000);
    }
  }
  sdp.startContinuous(true);
  sdp.startContinuousWait(true);
  
}

void loop() 
{
  int counter_0 = 0;
  int counter_1 = 0;
  float flow_1 = 10;
  float flow_2 = 10;

  Wire.beginTransmission(MUX_ADDR);
  for(int i = 0; i < 10; i++)
  {
    //    cmd[0] = 1 << 0;
    //    I2CHelper::i2c_write(MUX_ADDR, cmd, 1);
    enableMuxPort(0);
    int ret_sfm3000_1 = sfm3000_1.read_sample();
    // Wire.endTransmission();

    //    cmd[0] = 1 << 1;
    //    I2CHelper::i2c_write(MUX_ADDR, cmd, 1);
    enableMuxPort(1);
    int ret_sfm3000_2 = sfm3000_2.read_sample();
    // Wire.endTransmission();

    if (sdp.readContinuous() == 0)
      dP = sdp.getDifferentialPressure();

    enableMuxPort(2);
    abp_30psi_1.update();
    enableMuxPort(3);
    abp_30psi_2.update();
    enableMuxPort(4);
    abp_5psi_1.update();
    enableMuxPort(5);
    abp_5psi_2.update();
    enableMuxPort(6);
    abp_5psi_3.update();
    
    if (ret_sfm3000_1 == 0 ) 
    {
      flow_1 = sfm3000_1.get_flow();
      counter_0 = counter_0 + 1;
    } 
    
    if (ret_sfm3000_2 == 0 ) 
    {
      flow_2 = sfm3000_2.get_flow();
      counter_1 = counter_1 + 1;
    }

    // delayMicroseconds(100);
    
  }

  /*
  SerialUSB.print( "completed " );
  SerialUSB.print( counter_0 );
  SerialUSB.print( " + " );
  SerialUSB.print( counter_1 );
  SerialUSB.print( " flow measurements - " );
  SerialUSB.print( flow_1 );
  SerialUSB.print( "  " );
  SerialUSB.print( flow_2 );
  SerialUSB.print( " pressure: ");
  SerialUSB.print( abp_5psi_2.pressure() );
  SerialUSB.print( " " );
  SerialUSB.println( abp_5psi_2.unit() );
  */
  
  SerialUSB.print( abp_30psi_1.pressure() );
  SerialUSB.print( " " );
  SerialUSB.print( abp_30psi_1.unit() );
  SerialUSB.print( "\t" );
  SerialUSB.print( abp_5psi_2.pressure() );
  SerialUSB.print( " " );
  SerialUSB.println( abp_5psi_2.unit() );

}

#define MUX_ADDR 0x70 //7-bit unshifted default I2C Address

//Enables a specific port number
boolean enableMuxPort(byte portNumber)
{
  
  byte settings = (1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return(true);
}

/***************************************************************************************************/
/********************************************* pwm_lib *********************************************/
/***************************************************************************************************/
void change_duty( 
  pwm_base& pwm_obj, 
  uint32_t pwm_duty, 
  uint32_t pwm_period 
) 
{ 
  uint32_t duty=pwm_obj.get_duty()+pwm_duty; 
  if(duty>pwm_period) duty=pwm_duty; 
  pwm_obj.set_duty(duty); 
}
