
#define ROT_TO_LINES    1189.875
#define ROT_TO_DEG      360.0
#define ROT_TO_RAD      (2*PI)
#define SEC_TO_RPM      60.0
#define MAX_IE          255.0

int32_t interrupt_count = 0;
int32_t count = 0;
int dir = 1;
double dt = 1.0/125.0;
int32_t current_count;
int32_t previous_count;
double current_position = 0;
double previous_position = 0;
double goal_position = 100;
double rounded_position = 0;
double current_rpm = 0;
double previous_rpm = 0;
double goal_rpm = 100.0;
double integral_error = 0;
double derivative_error = 0;
double gain = 15.0;
double output = 0;
double control = 0;

double K_p = 0.1;
double K_i = 0.01;
double K_d = 0.001;

ISR(TIMER0_COMPA_vect) {
  previous_count = current_count;
  current_count = count;
  previous_rpm = current_rpm;
  current_rpm = ((double)(current_count - previous_count)*SEC_TO_RPM)/(dt*ROT_TO_LINES);
  
  previous_position = current_position;
  current_position = (current_count*ROT_TO_DEG)/ROT_TO_LINES;
  
  // Take into account the rotational constraints
  rounded_position = ((((int32_t)current_count)%((int32_t)ROT_TO_LINES))*ROT_TO_DEG)/ROT_TO_LINES;  
  
  double error = goal_position - current_position;
  if(error > 180) {
    error = 360 - error;  
  }
  else if(error < -180) {
    error = -360 - error;  
  }
  integral_error += error*dt;
  if(integral_error > MAX_IE/K_i) integral_error = MAX_IE;
  if(integral_error < -MAX_IE/K_i) integral_error = -MAX_IE;
  derivative_error = (current_position - previous_position)/dt;
  
  output = gain*(K_p*error + K_i*integral_error + K_d*derivative_error);
  control = output;
  
  if(control < 0) {
    digitalWrite(12, LOW);
    analogWrite(11, constrain((int)-control, 0, 255));  
  }
  else {
    digitalWrite(12, HIGH);
    analogWrite(11, constrain((int)control, 0, 255));      
  }
  interrupt_count++;
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
  TCCR0B |= (1 << CS02);// | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();
  
  Serial.begin(9600);
  pinMode(11,OUTPUT);  
  pinMode(12,OUTPUT);
  pinMode(2,INPUT);  
  pinMode(3,INPUT);   
  digitalWrite(12, HIGH);
  analogWrite(11, control);
  attachInterrupt(0,encoder_counter,RISING);
}

void loop()
{
  if(Serial.available()) {
    // Write code to read goal angular position in DEG
    char cmd = Serial.read();
    if(cmd == 'f') digitalWrite(12, HIGH);
    else if(cmd == 'b') digitalWrite(12, LOW);
  }
  else {
    long_delay();
    Serial.print((double)current_count);
    Serial.print(" ");
    Serial.print(rounded_position);
    Serial.print(" ");
    Serial.println(output);
    //Serial.println((double)interrupt_count);
  }
}

void long_delay()
{
  for(int i = 0; i < 10; i++) delayMicroseconds(100000000);
}
