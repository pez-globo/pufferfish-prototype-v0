// Arduino class for a quadrature encoder

/*
  quadratureEncoder.h - Library for reading a quadrature encoder. 
  Created by Deepak Krishnamurthy, May 12, 2020.

  Reads the quadrature encoder signal from a pair of digital pins.

  Encoder signal is read using Interrupts.

  Decodes the signal to update the Encoder Tick Counts.
  
*/

#ifndef quadratureEncoder_h
#define quadratureEncoder_h

#include "Arduino.h"

class quadratureEncoder
{
	public:

		quadratureEncoder(int pin_A, int pin_B, int mode);

		void decoder(bool EncoderAPrev, bool EncoderBPrev, bool EncoderASet, bool EncoderBSet);

		void HandleInterrupt();

		volatile long int EncoderTicks;
		volatile int Dir;
		

	private:

		volatile bool _EncoderASet;
		volatile bool _EncoderBSet;
		volatile bool _EncoderAPrev;
		volatile bool _EncoderBPrev;

}


#endif