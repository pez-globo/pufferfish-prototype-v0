#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>

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

static const int pin_valve1 = 48;
static const int pin_valve2 = 49;

#include <DueTimer.h>
static const int TIMER_PERIOD = 500; // in us

void setup() {

  // Initialize Native USB port
  //SerialUSB.begin(2000000);     
  //while(!SerialUSB);            // Wait until connection is established
  buffer_rx_ptr = 0;

  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  
  pinMode(pin_valve1, OUTPUT);
  digitalWrite(pin_valve1, LOW);

  pinMode(pin_valve2, OUTPUT);
  digitalWrite(pin_valve2, LOW);

  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD); 
  
}

void timer_interruptHandler()
{
  digitalWrite(13,LOW);
}


void loop() 
{
  while (SerialUSB.available()) 
  { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) 
    {
      buffer_rx_ptr = 0;
      if(buffer_rx[0]==3)
        set_valve1_state(buffer_rx[1]);
      else if(buffer_rx[0]==4)
        set_valve2_state(buffer_rx[1]);
    }
  }
}

void set_valve1_state(int state)
{
  if(state>0)
    digitalWrite(pin_valve1, HIGH);
  else
    digitalWrite(pin_valve1, LOW);
}

void set_valve2_state(int state)
{
  if(state>0)
    digitalWrite(pin_valve2, HIGH);
  else
    digitalWrite(pin_valve2, LOW);
}

// utils
long signed2NBytesUnsigned(long signedLong,int N)
{
  long NBytesUnsigned = signedLong + pow(256L,N)/2;
  //long NBytesUnsigned = signedLong + 8388608L;
  return NBytesUnsigned;
}
