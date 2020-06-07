#include <DueTimer.h>

static const int MSG_LENGTH = 960;
byte buffer_tx[MSG_LENGTH];
static const int TIMER_PERIOD_us = 1250; // in us
volatile uint16_t tmp = 0;
volatile int counter_send_data = 0;
volatile bool flag_send_data = false;

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established

  delayMicroseconds(1000000);

  // start the timer
  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

}

void timer_interruptHandler()
{
  // send data to host computer
  counter_send_data = counter_send_data + 1;
  if (counter_send_data*TIMER_PERIOD_us >= 50*1000 )
  {
    counter_send_data = 0;
    flag_send_data = true;
  }
  tmp = (tmp + 1) % int(1000/1.25);
  
}

void loop()
{
  if (flag_send_data)
  {
    buffer_tx[0] = byte(tmp >> 8);
    buffer_tx[1] = byte(tmp % 256);
    SerialUSB.write(buffer_tx, MSG_LENGTH);
    flag_send_data = false;
  }

}
