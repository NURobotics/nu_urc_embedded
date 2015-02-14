
volatile int count = 0;
volatile int dir = 1;
volatile int preverr;
volatile int err;
volatile float rpm=0;
volatile float grpm = 128;
volatile int prevc = 0;
volatile int dcount;
volatile float rev;
volatile float deriv;
volatile float integ = 0;
volatile float output;
float kp = .5;
float ki = 0;
float kd = 0;
int c = 0;

ISR(TIMER0_COMPA_vect){
  dcount = count - prevc;
  rev = dcount/(344*4.25);
  rpm = rev*60000;
  err = grpm - rpm;
  integ = integ + err*.001;
  deriv = (err - preverr)/.001;
  output = kp*err+ki*integ+kd*deriv;
  preverr = err;
  prevc = count;
  c ++;
}

void encoder_counter()
{
  int x = PIND;
  if(x&(0b1000)) dir = 1;
  else dir = -1;
  count = count + dir;
}

void setup()
{
  cli();
//set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 248;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  
  sei();
  
  Serial.begin(9600);
  pinMode(11,OUTPUT);  
  pinMode(12,OUTPUT);
  pinMode(2,INPUT);  
  pinMode(3,INPUT);    
  attachInterrupt(0,encoder_counter,RISING);
}

void loop()
{
  digitalWrite(12, HIGH);
  analogWrite(11, 90);// + output);
  delayMicroseconds(1000000);
  Serial.println(rpm);  
}
