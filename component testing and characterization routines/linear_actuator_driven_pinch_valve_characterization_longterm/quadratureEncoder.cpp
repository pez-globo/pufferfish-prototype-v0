// Arduino class for a quadrature encoder

/*
  quadratureEncoder.h - Library for reading a quadrature encoder. 
  Created by Deepak Krishnamurthy, May 12, 2020.
  
*/
#include "Arduino.h"
#include "quadratureEncoder.h"

quadratureEncoder::quadratureEncoder(int pin_A, int pin_B, int trigger_mode, int pin_mode = INPUT_PULLUP)
{
	// Set the pin modes as INPUT
	pinMode(pin_A, pin_mode);
  	pinMode(pin_B, pin_mode);

	attachInterrupt(digitalPinToInterrupt(pin_A), HandleInterrupt, trigger_mode);


}
		

	

