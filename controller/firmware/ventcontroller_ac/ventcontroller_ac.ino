//#include <TMCStepper.h>
//#include <TMCStepper_UTILITY.h>
#include <Wire.h>
// https://github.com/ethanjli/arduino-sdp
#include <sdpsensor.h>
#include <HoneywellTruStabilitySPI.h>

// HSCMRRN060MDSA3
//  - M       SMT
//  - RR      dual radial barbed ports, same side
//  - N       dry gas only, no diagnostics
//  - 060MD   ±60 mbar
//  - S       SPI
//  - A       2e14 counts
//  - 3       3.3 Vcc

#define SLAVE_SELECT_PIN 0
TruStabilityPressureSensor pressure_sensor( SLAVE_SELECT_PIN, -61.183, 61.183 ); // unit: cmH2O
SDP8XXSensor sdp;

/*
  // create Honeywell_ABP instance
  Honeywell_ABP abp(
  0x28,   // I2C address
  0,      // minimum pressure
  100,      // maximum pressure
  "mbar"   // pressure unit
  );
*/

static inline int sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

static const int CMD_LENGTH = 4;
static const int MSG_LENGTH = 8;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
static const int N_BYTES_POS = 3;

static const int pin_valve1 = 30;
static const int pin_valve2 = 31;

static const float flow_FS = 100;
static const float volume_FS = 800;
static const float paw_FS = 60;
static const float Ti_FS = 5;
static const float Vt_FS = 800;
static const float PEEP_FS = 30;
static const float RR_FS = 60;

static const float p1 = 0.0013; // correction coefficient
static const float p2 = 0.0687; // correction coefficient
static const float p3 = -0.2016; // correction coefficient

//static const float p1 = 0; // correction coefficient
//static const float p2 = 0; // correction coefficient
//static const float p3 = 0; // correction coefficient

static const uint8_t CMD_Vt = 0;
static const uint8_t CMD_Ti = 1;
static const uint8_t CMD_RR = 2;
static const uint8_t CMD_PEEP = 3;
static const uint8_t CMD_Flow = 4;
static const uint8_t CMD_FlowDeceleratingSlope = 5;
static const uint8_t CMD_valve1 = 10;
static const uint8_t CMD_valve2 = 11;

static const float coefficient_dP2flow = 0.6438;
static const float coefficient_dp2flow_offset = 1.1029;

volatile float dP = 0;
volatile float flow = 0;
volatile float volume = 0;
volatile float paw = 0;

float RR = 18;
float Ti = 1.2;
float Vt = 300;
float PEEP = 5;
float paw_trigger_th = 2;
//float paw_trigger_th = -3;

float cycle_period_ms = 0; // duration of each breathing cycle
float cycle_time_ms = 0;  // current time in the breathing cycle
float time_inspiratory_ms = Ti * 1000;
float frequency_send_data = 25;
float counter_send_data = 0;

volatile bool flag_send_data = false;
volatile bool flag_read_sensor = false;

volatile bool is_in_inspiratory_phase = false;
volatile bool is_in_expiratory_phase = false;
volatile bool PEEP_is_reached = false;

uint16_t tmp_uint16;
int16_t tmp_int16;
long tmp_long;

uint16_t timebase = 0; // in number of TIMER_PERIOD_us
static const long DISPLAY_RANGE_S = 20;

#include <DueTimer.h>
static const float TIMER_PERIOD_us = 2500; // in us

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established
  buffer_rx_ptr = 0;

  SPI.begin(); // start SPI communication
  pressure_sensor.begin(); // run sensor initialization

  Wire.begin();
  sdp.stopContinuous(); // stop continuous measurement if it's running
  // init sensirion sensor
  while (true)
  {
    int ret = sdp.init();
    if (ret == 0)
      break;
    else
      delay(100);
  }
  sdp.startContinuous(true);
  sdp.startContinuousWait(true);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(pin_valve1, OUTPUT);
  digitalWrite(pin_valve1, LOW);

  pinMode(pin_valve2, OUTPUT);
  digitalWrite(pin_valve2, LOW);

  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);

  cycle_period_ms = (60 / RR) * 1000;

}

void timer_interruptHandler()
{
  timebase = timebase + 1;
  if (timebase >= (DISPLAY_RANGE_S * 1000000 / TIMER_PERIOD_us))
    timebase = 0;

  // read sensor value
  flag_read_sensor = true;

  // update cycle timer
  cycle_time_ms = cycle_time_ms + TIMER_PERIOD_us / 1000;

  // time-triggered breath
  if (cycle_time_ms > cycle_period_ms)
  {
    cycle_time_ms = 0;
    is_in_inspiratory_phase = true;
    is_in_expiratory_phase = false;
    PEEP_is_reached = false;
    volume = 0;
    set_valve2_state(0);
    set_valve1_state(1);
    digitalWrite(13, HIGH);
  }

  // patient triggered breath
  if ( paw < paw_trigger_th && is_in_expiratory_phase)
  {
    cycle_time_ms = 0;
    is_in_inspiratory_phase = true;
    is_in_expiratory_phase = false;
    PEEP_is_reached = false;
    volume = 0;
    set_valve2_state(0);
    set_valve1_state(1);
    digitalWrite(13, HIGH);
  }

  // breathing control - stop inspiratory flow when Vt is reached
  //if (volume >= Vt-35) // compensate for system response time
  if (volume >= Vt-30) // compensate for system response time, for 10 psi
    set_valve1_state(0);

  // breathing control - change to exhalation when Ti is reached
  if (cycle_time_ms > time_inspiratory_ms)
  {
    digitalWrite(13, LOW);

    is_in_inspiratory_phase = false;
    is_in_expiratory_phase = true;
    set_valve1_state(0);
    // only allow expiratory flow when Paw is >= PEEP
    // if (paw > PEEP && PEEP_is_reached == false)
    if (paw  > PEEP && PEEP_is_reached == false) // take into account of flow induced pressure
      set_valve2_state(1);
    else
    {
      set_valve2_state(0);
      PEEP_is_reached = true;
    }
  }

  // send data to host computer
  counter_send_data = counter_send_data + 1;
  if ((TIMER_PERIOD_us / 1000000)*counter_send_data >= 1 / frequency_send_data)
  {
    counter_send_data = 0;
    flag_send_data = true;
  }

}

void loop()
{
    while(SerialUSB.available())
    {
      buffer_rx[buffer_rx_ptr] = SerialUSB.read();
      buffer_rx_ptr = buffer_rx_ptr + 1;
      if (buffer_rx_ptr == CMD_LENGTH)
      {
        buffer_rx_ptr = 0;
        if(buffer_rx[0]==CMD_Vt)
          Vt = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*Vt_FS;
        else if(buffer_rx[0]==CMD_Ti)
        {
          Ti = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*Ti_FS;
          time_inspiratory_ms = Ti*1000;
        }
        else if(buffer_rx[0]==CMD_RR)
        {
          RR = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*RR_FS;
          cycle_period_ms = (60/RR)*1000;
        }
        else if(buffer_rx[0]==CMD_PEEP)
          PEEP = ((256*float(buffer_rx[1])+float(buffer_rx[2]))/65536)*PEEP_FS;
        else if(buffer_rx[0]==CMD_valve1)
          set_valve1_state(buffer_rx[1]);
        else if(buffer_rx[0]==CMD_valve2)
          set_valve2_state(buffer_rx[1]);
      }
    }

  if (flag_read_sensor)
  {
    if (pressure_sensor.readSensor() == 0)
      paw = pressure_sensor.pressure();
    if (sdp.readContinuous() == 0)
      dP = sdp.getDifferentialPressure();
    if(abs(dP)<0.3)
      flow = 0;
    else if(dP >=3 )
      flow = dP * coefficient_dP2flow + coefficient_dp2flow_offset;
    else
      flow = dP * coefficient_dP2flow - coefficient_dp2flow_offset;

    // correct for pressure drop
    paw = paw - sgn(flow)*((p1*flow*flow) + p2*abs(flow) + p3);
    
    volume = volume + flow * 1000 * (TIMER_PERIOD_us / 1000000 / 60);
    flag_read_sensor = false;
  }

  if (flag_send_data)
  {
    tmp_long = (65536 / 2) * paw / paw_FS;
    // tmp_long = (65536 / 2) * (paw + alpha*flow*flow) / paw_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[0] = byte(tmp_uint16 >> 8);
    buffer_tx[1] = byte(tmp_uint16 % 256);

    tmp_long = (65536 / 2) * flow / flow_FS;
    tmp_uint16 = signed2NBytesUnsigned(tmp_long, 2);
    buffer_tx[2] = byte(tmp_uint16 >> 8);
    buffer_tx[3] = byte(tmp_uint16 % 256);

    tmp_uint16 = 65536 * volume / volume_FS;
    buffer_tx[4] = byte(tmp_uint16 >> 8);
    buffer_tx[5] = byte(tmp_uint16 % 256);

    buffer_tx[6] = byte(timebase >> 8);
    buffer_tx[7] = byte(timebase % 256);

     SerialUSB.write(buffer_tx,MSG_LENGTH);
     flag_send_data = false;

    //    SerialUSB.print(paw);
    //    SerialUSB.print("\t ");
    //    SerialUSB.print(dP);
    //    SerialUSB.print("\t ");
    //    SerialUSB.println(flow);
    //    SerialUSB.println(volume);

  }
}

void set_valve1_state(int state)
{
  if (state > 0)
    digitalWrite(pin_valve1, HIGH);
  else
    digitalWrite(pin_valve1, LOW);
}

void set_valve2_state(int state)
{
  if (state > 0)
    digitalWrite(pin_valve2, HIGH);
  else
    digitalWrite(pin_valve2, LOW);
}

// utils
long signed2NBytesUnsigned(long signedLong, int N)
{
  long NBytesUnsigned = signedLong + pow(256L, N) / 2;
  return NBytesUnsigned;
}
