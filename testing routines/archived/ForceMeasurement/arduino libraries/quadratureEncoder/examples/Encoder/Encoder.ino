
#define Encoder_Pin_A 28            // Pin to read encoder channel A 
#define Encoder_Pin_B 29            // Pin to read encoder channel B 

volatile bool _Encoder_A_Set;
volatile bool _Encoder_B_Set;
volatile bool _Encoder_A_Prev;
volatile bool _Encoder_B_Prev;
volatile int Dir;
volatile long int _EncoderTicks = 0;

void setup() 
{
  // put your setup code here, to run once:
  SerialUSB.begin(9600);

   // ENCODERS

  pinMode(Encoder_Pin_A, INPUT);
  pinMode(Encoder_Pin_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_Pin_A), HandleInterrupt, CHANGE);

}

int Decoder(bool Encoder_A_Prev, bool Encoder_B_Prev, bool Encoder_A_Set, bool Encoder_B_Set)
{
  if(!Encoder_A_Prev && Encoder_A_Set)
  {
    if(Encoder_B_Set) return -1;
    else return 1;
  }
  else if(Encoder_A_Prev && !Encoder_A_Set)
  {
    if(!Encoder_B_Set) return -1;
    else return 1;
  }
  else return 0;
  
}
void HandleInterrupt()
{

    _Encoder_B_Set = digitalRead(Encoder_Pin_B);
    _Encoder_A_Set = digitalRead(Encoder_Pin_A);

   //Dir=ParseEncoder();
    Dir = Decoder(_Encoder_A_Prev, _Encoder_B_Prev, _Encoder_A_Set, _Encoder_B_Set);
    _EncoderTicks += Dir;
  

    _Encoder_A_Prev = _Encoder_A_Set;
    _Encoder_B_Prev = _Encoder_B_Set;

}

void loop() {
  // put your main code here, to run repeatedly:

  SerialUSB.println(_EncoderTicks);
//  SerialUSB.println(Dir);
  
}
