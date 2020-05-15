#include <quadratureEncoder.h>

quadratureEncoder encoder1(22,24, CHANGE, INPUT_PULLUP);
void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  SerialUSB.print(encoder1.EncoderTicks);
  
}
