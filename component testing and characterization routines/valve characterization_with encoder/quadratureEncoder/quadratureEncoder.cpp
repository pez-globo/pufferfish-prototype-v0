// Arduino class for a quadrature encoder

/*
  quadratureEncoder.h - Library for reading a quadrature encoder. 
  Created by Deepak Krishnamurthy, May 12, 2020.
  
*/
#include "Arduino.h"
#include "quadratureEncoder.h"

quadratureEncoder::quadratureEncoder(int pin_A, int pin_B, int trigger_mode = CHANGE, int pin_mode = INPUT_PULLUP)
{	
	/*
	Pin A, Pin B: the digital pins connected to the encoder
	trigger_mode: What triggers the interrupt: RISING, FALLING, CHANGE
	pin_mode: Pin mode for the digital pins, needs to be INPUT or INPUT_PULLUP
	*/
	// Set the pin modes as INPUT or INPUT_PULLUP

	_pin_A = pin_A;
	_pin_B = pin_B;

	// if(pin_mode == INPUT || pin_mode == INPUT_PULLUP)
	// {
  pinMode(_pin_A, pin_mode);
	pinMode(_pin_B, pin_mode);
  	// }
  	// else
  	// 	throw "Invalid pin mode!";

  	// Attach an interrupt handling function to Pin A 
	attachInterrupt(digitalPinToInterrupt(_pin_A), HandleInterrupt, trigger_mode);


}


int quadratureEncoder::Decoder(bool EncoderAPrev, bool EncoderBPrev, bool EncoderASet, bool EncoderBSet)
{
	if(!EncoderAPrev && EncoderASet)
  	{
	    if(EncoderBSet) return -1;
	    else return 1;
  	}
  	else if(EncoderAPrev && !EncoderASet)
  	{
    	if(!EncoderBSet) return -1;
    	else return 1;
  	}
  	else return 0;
}

void quadratureEncoder::HandleInterrupt()
{
	_EncoderBSet = digitalRead(_pin_B);
    _EncoderASet = digitalRead(_pin_A);

    Dir = Decoder(_EncoderAPrev, _EncoderBPrev, _EncoderASet, _EncoderBSet);
    EncoderTicks += Dir;
  
    _EncoderAPrev = _EncoderASet;
    _EncoderBPrev = _EncoderBSet;
}

	

