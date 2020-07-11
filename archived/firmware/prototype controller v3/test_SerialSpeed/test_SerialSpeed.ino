static const int MSG_LENGTH = 24;
byte buffer_tx[MSG_LENGTH];

void setup() 
{
  // put your setup code here, to run once:
  SerialUSB.begin(2000000);
  while (!SerialUSB);  

  delayMicroseconds(200000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //SerialUSB.println("starting transmitting 10000 24-byte messages");
  for(int i = 0;i < 10000;i++)
  {
    SerialUSB.write(buffer_tx, MSG_LENGTH);
  }
  SerialUSB.println("finished transmitting 10000 24-byte messages");
}
