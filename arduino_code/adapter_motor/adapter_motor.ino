//***********************************************************************//
/*
Aquanaute - PWM interpreter
Code to read a pwm exit of the navio2 to transform it to a byte speed command, to control a continuous drive motor with an absolute encoder.
Made by: Ricardo RICO URIBE - Internt at U2IS during the summer of 2020
Helpers:
- Thibault TORALBA
- Thomas SIMON
*/
//***** CONSTANTS *****//
const byte INTERRUPT_NOISE_THRESHOLD = 10; //microseconds in which the detection of interrupt will ignore a change

const word MOTOR_CENTER = 1000;  //motor position center, this was arbitrarily chosen
const word MOTOR_RIGHT = 1300;   //300 steps, arbitrarily chosen to indicate full right, this is aprox 90°
const word MOTOR_LEFT = 700;     //300 steps, arbitrarily chosen to indicate full left, this is aprox 90°
const byte MOTOR_THRESHOLD = 10; //value to range the motor position to eliminate oscilation
const word MOTOR_RANGE = MOTOR_RIGHT - MOTOR_LEFT;

const word RADIO_CENTER = 1492;  //radio "high time" when radio control is centered, experimentaly obtained, directly linked with the trim value of ardupilot params
const word RADIO_RIGHT = 1872;   //radio "high time" when radio control is full right, experimentaly obtained, directly linked with the max pwm value of ardupilot params
const word RADIO_LEFT = 1120;    //radio "high time" when radio control is full left, experimentaly obtained, directly linked with the min pwm value of ardupilot params
const byte RADIO_THRESHOLD = 10; //value to range the time calculated to eliminate oscilation
const word RADIO_RANGE = RADIO_RIGHT - RADIO_LEFT;

const byte COMM_ALL_STOP = 0;
const byte COMM_STOP = 64;
const byte COMM_FULL_FWD = 127;
const byte COMM_MED_FWD = 100;
const byte COMM_FULL_BCK = 1;
const byte COMM_MED_BCK = 28;
//***** Variables *****//
byte speed_command = COMM_ALL_STOP; //Message to send to the Motor Controller

const int _kp = 1; //Controller Proportional Coefficient

const byte interrupt_pin = 20;                    //Pin to detect interruptions (connected to the yaw channel in the Navio)
static volatile uint32_t delta_us;                //time "high"
static volatile uint32_t prev_change_date_us = 0; //record previous time
static volatile uint32_t pos = 0;                 //motor position
static volatile byte count = 0;                   //counter for position response bytes (0-3)
static volatile uint32_t recv = 0;                //Bytes recieved

bool first = true;                    //first byte recieved
bool started = false;                 //setup to center the motor
bool go_print = false;                //debug routine
static volatile int scale_pos = 50;   //scale of position, initialized as centered
static volatile int scale_radio = 50; //scale of radio controller position, initialized as centered
//***********************************************************************//
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);    //usb pc
  Serial1.begin(9600);   //sabertooth driver (speed selected physically on the driver)
  Serial2.begin(115200); //multiplexer (speed recommended by the manufactor)
  Serial1.write(speed_command);
  while (!Serial)
  {
    //wait for pc connection or RP4 power_up
  }
  cli(); //stop interrupts

  //setup for timed interrupts
  TCCR4A = 0; // set entire TCCR4A register to 0
  TCCR4B = 0; // same for TCCR4B
  TCNT4 = 0;  //initialize counter value to 0
  // set compare match register for 20hz increments
  OCR4A = 311 / 1; // = (16*10^6) / (50*1024) - 1 (must be <65536) // 311 = 50Hz / // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);

  //Setup for external Interrupt
  sei();                                                              //allow interrupts
  pinMode(interrupt_pin, INPUT_PULLUP);                               //activate internal resistor for interrupt pin
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), pwm, CHANGE); //set pin to read external interrupts
} //end setup
//***********************************************************************//
//*****Time Interrupt - Request the Motor Position*****//
/*Request the motor position, with a frecuency of 50Hz*/
ISR(TIMER4_COMPA_vect)
{
  Serial2.write(0x43); //Char "C", by design this is the message that the encoder expects to respond with its position
  count = 0;           //reset of message counter
  first = true;        //reset of first message
}
//***********************************************************************//
//*****Physical Interruption - Read of Navio PWM*****//
/*Read a change from low to high or from high to low in the pwm signal sended by the navio,
and obtain the "high" time to interpret it as duty cycle and to translate it to and scale between 0 and 100,
where 0 is full left, 50 is center, and 100 is full right*/
void pwm()
{
  uint32_t curr_date = micros();
  bool state_us;
  /* Record the previous pin state since it has just changed. */
  state_us = !digitalRead(interrupt_pin);
  /* Ensure that we have detected a falling edge, i.e. the previous level of the pin was HIGH. */
  if (state_us)
  {
    delta_us = curr_date - prev_change_date_us;
    if (delta_us < INTERRUPT_NOISE_THRESHOLD || delta_us < RADIO_LEFT - RADIO_THRESHOLD || delta_us > RADIO_RIGHT + RADIO_THRESHOLD)
      return;
    if (delta_us <= RADIO_CENTER + RADIO_THRESHOLD && delta_us >= RADIO_CENTER - RADIO_THRESHOLD)
    {
      scale_radio = 50;
    }
    else if (delta_us <= RADIO_LEFT + RADIO_THRESHOLD)
    {
      scale_radio = 0;
    }
    else if (delta_us >= RADIO_RIGHT - RADIO_THRESHOLD)
    {
      scale_radio = 100;
    }
    else
    {
      scale_radio = (delta_us - RADIO_LEFT) * 100 / (RADIO_RANGE);
    }
    go_print = true;
  }
  else
  {
    prev_change_date_us = curr_date;
  }
}
//*****Serial Interrupt - Read Motor Position*****//
/*If the encoder answers with its position, save the 4 bytes of information, translate them to decimal
and do a convertion to be in a scale between 0 and 100, where 0 is full left, 50 is center and 100 is full right.
After that, do a proportial control and send an speed command to the motor, the command is coded in 7 bits,
where 1 is full backward, 64 is stop and 127 is full forward*/
void serialEvent2()
{
  while (Serial2.available())
  {
    if (first) //Save the first byte of the message
    {
      recv = Serial2.read();
      first = false;
    }
    else //chain all the bytes to have the number that indicates a position
    {
      recv = (recv << 8) + Serial2.read();
    }
    count++;
  }
  if (count == 4) //when all 4 bytes of the message have been recieved, do the control
  {
    pos = recv;
    count = 0;
    speed_command = COMM_ALL_STOP;
    first = true;

    if (!started) //When the arduino recieves power it will center the motor.
    {
      if (pos > MOTOR_CENTER + MOTOR_THRESHOLD || pos < MOTOR_CENTER - MOTOR_THRESHOLD)
      {
        if (pos < MOTOR_LEFT)
          speed_command = COMM_FULL_FWD;
        else if (pos > MOTOR_RIGHT)
          speed_command = COMM_FULL_BCK;
        else if (pos < MOTOR_CENTER - MOTOR_THRESHOLD)
          speed_command = COMM_MED_FWD;
        else if (pos > MOTOR_CENTER + MOTOR_THRESHOLD)
          speed_command = COMM_MED_BCK;
      }
      else
      {
        Serial.print("centered");
        speed_command = COMM_STOP;
        started = true;
      }
      Serial1.write(speed_command);
    }
    else //Normal use
    {
      scale_pos = (pos - MOTOR_LEFT) * 100 / (MOTOR_RANGE); //scale the motor position to a range between 0,100

      int error_pos = scale_radio - scale_pos;

      int v = _kp * error_pos; //Control

      if (v >= -1 && v <= 1) //if the error is at 0 with a threshold, send stop to the motor
      {
        speed_command = COMM_STOP;
      }
      else if (v >= 30) //if the error (-50,50) is more than "30" (value arbitrarily chosen) send the maximum speed, this is done to minimize the time of the mouvement
      {
        speed_command = COMM_FULL_FWD;
      }
      else if (v <= -30) //if the error (-50,50) is less than "-30" (value arbitrarily chosen) send the maximum speed, this is done to minimize the time of the mouvement
      {
        speed_command = COMM_FULL_BCK;
      }
      else //Calculate the speed with this formula that transforms the error (-50,50) into the speed command (1,127)
      {
        speed_command = (1.26) * (v - 50) + 127;
      }
    }
    go_print = true;
    Serial1.write(speed_command);
  }
}
//***********************************************************************//
void loop()
{
  //routine for debug. "never print inside an interruption"
  if (go_print)
  {
    //Serial.print("delta time: ");
    //Serial.println(delta_us);
    //Serial.print("speed command: ");
    //Serial.println(speed_command);
  }
}
