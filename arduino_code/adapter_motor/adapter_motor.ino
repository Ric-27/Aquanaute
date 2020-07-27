//timer setup for timer4
//For arduino Mega

//timer4 will interrupt at 20Hz

//storage variable
const byte INTERRUPT_NOISE_THRESHOLD = 5 ;

const word MOTOR_CENTER = 1000 ;
const word MOTOR_RIGHT = 1300 ;
const word MOTOR_LEFT = 700 ;
const byte MOTOR_THRESHOLD = 10 ;
const word MOTOR_RANGE = MOTOR_RIGHT - MOTOR_LEFT ;

const word RADIO_CENTER = 1500 ;
const word RADIO_RIGHT = 1900 ;
const word RADIO_LEFT = 1100 ;
const byte RADIO_THRESHOLD = 10 ;
const word RADIO_RANGE = RADIO_RIGHT - RADIO_LEFT ;

const int _kp = 1;

const byte interrupt_pin = 2 ;
static volatile uint32_t delta_us ;
static volatile uint32_t prev_change_date_us = 0 ;
static volatile uint32_t pos = 0;
static volatile byte count = 0;
static volatile uint32_t recv = 0;

bool first = true;
bool started = false;
static volatile int scale_pos = 50;
static volatile int scale_radio = 50;
byte speed_command = 0;
//***********************************************************************//
void setup(){
// put your setup code here, to run once:
  Serial.begin(9600); //usb pc
  Serial1.begin(9600); //sabertooth driver
  Serial2.begin(115200); //multiplexer
  Serial1.write(speed_command);
  while(!Serial){
    
  }
  cli(); //stop interrupts
  
  //set timer4 interrupt at 20Hz
  TCCR4A = 0;// set entire TCCR4A register to 0
  TCCR4B = 0;// same for TCCR4B
  TCNT4  = 0;//initialize counter value to 0
  // set compare match register for 20hz increments
  OCR4A = 311/1;// = (16*10^6) / (20*1024) - 1 (must be <65536) // 780 = 200hz / // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);
  
  sei();//allow interrupts

  attachInterrupt(digitalPinToInterrupt(interrupt_pin),pwm,CHANGE);
  //delay(5);
}//end setup

ISR(TIMER4_COMPA_vect){
  //Serial.print("radio: ");
  //Serial.println(scale_radio);
  Serial2.write(0x43);
  count = 0;
  first = true;
}
//***********************************************************************//
void loop(){
 //do other things here
}
//***********************************************************************//
void pwm() {
  //Serial.print("change");
  uint32_t curr_date = micros () ;
  bool state_us ;

  delta_us = curr_date - prev_change_date_us ;
  
  if (delta_us < INTERRUPT_NOISE_THRESHOLD) return ;
  /* Record the previous pin state since it has just changed. */
  state_us = ! digitalRead (interrupt_pin) ;
  /* Ensure that we have detected a falling edge, i.e. the previous level
     of the pin was HIGH. */
  if (state_us) {
    //Serial.print("radio: ");
    //Serial.println(delta_us);
    if (delta_us <= RADIO_CENTER + RADIO_THRESHOLD && delta_us >= RADIO_CENTER - RADIO_THRESHOLD){
      scale_radio = 50;
    }
    else if (delta_us <= RADIO_LEFT + RADIO_THRESHOLD){
      scale_radio = 0;
    }
    else if (delta_us >= RADIO_RIGHT - RADIO_THRESHOLD){
      scale_radio = 100;
    } else {
      scale_radio = (delta_us - RADIO_LEFT) * 100 / (RADIO_RANGE);
    }
    //Serial.print("scale_radio: ");
    //Serial.println(scale_radio);
  }
  /* Change the date anyway. */
  prev_change_date_us = curr_date ;
}
//***********************************************************************//
void serialEvent2() {
  while(Serial2.available()){
    if (first) {
      recv = Serial2.read();
      first = false;
    }
    else {
      recv = (recv << 8) + Serial2.read();
    }
    count++;
  }
  if (count == 4) {
    pos = recv;
    //Serial.print("scale_pos: ");
    //Serial.println(pos);
    count = 0;
    speed_command = 64;
    first = true;
    if (!started) {
      if (pos > MOTOR_CENTER + MOTOR_THRESHOLD || pos < MOTOR_CENTER - MOTOR_THRESHOLD){
        if(pos < MOTOR_LEFT) speed_command = 127;
        else if(pos > MOTOR_RIGHT) speed_command = 1;
        else if(pos < MOTOR_CENTER - MOTOR_THRESHOLD) speed_command = 100;
        else if(pos > MOTOR_CENTER + MOTOR_THRESHOLD) speed_command = 28;
      } else {
        //Serial.print("centered");
        speed_command = 64;
        started = true;
      }
      Serial1.write(speed_command);
    }
    else {
      scale_pos = (pos - MOTOR_LEFT)* 100/(MOTOR_RANGE);
      
      //Serial.print("scale_pos: ");
      //Serial.println(scale_pos);
      
      int error_pos = scale_radio - scale_pos;
      int v = _kp * error_pos;
      if (v >= -1 && v <= 1){
        speed_command = 64;
      }
      else if (v >= 30) {
        speed_command = 127;
      }
      else if (v <= -30) {
        speed_command = 1;
      }
      else {
        speed_command = (1.26) * (v - 50) + 127;
      }
      //Serial.print("v: ");
      //Serial.println(v);
      //Serial.print("speed_command: ");
      //Serial.println(speed_command);
    }
    Serial1.write(speed_command);
  }
}
//***********************************************************************//
void serialEvent(){
  started = true;
  Serial1.write(0);
}
