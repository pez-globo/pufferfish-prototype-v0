#include "pwm_lib.h"

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

uint32_t tmp = 0;

void setup() 
{
  // starting PWM signals
  pwm_wrapper_pin35_ptr->start(PWM_PERIOD_PIN_35,PWM_DUTY_PIN_35);
  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  //  delayMicroseconds(100000);
  //  tmp = (tmp + 500) % PWM_PERIOD_PIN_35;
  //  change_duty(*pwm_wrapper_pin35_ptr,tmp,PWM_PERIOD_PIN_35);
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
