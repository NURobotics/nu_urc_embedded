#include <RoverDriveState.h>

namespace nu_urc_embedded {

volatile unsigned int* OCxCON[NUM_OC_PORTS] = {&OC1CON, &OC2CON, &OC3CON, &OC4CON, &OC5CON};
volatile unsigned int* OCxRS[NUM_OC_PORTS]  = {&OC1RS, &OC2RS, &OC3RS, &OC4RS, &OC5RS};
volatile unsigned int* OCxR[NUM_OC_PORTS]   = {&OC1R, &OC2R, &OC3R, &OC4R, &OC5R};
volatile unsigned int* TxCON[NUM_TIMERS]    = {&T1CON, &T2CON, &T3CON, &T4CON, &T5CON};
volatile unsigned int* TMRx[NUM_TIMERS]     = {&TMR1, &TMR2, &TMR3, &TMR4, &TMR5};
volatile unsigned int* PRx[NUM_TIMERS]      = {&PR1, &PR2, &PR3, &PR4, &PR5};

bool PWM::reset(int oc, int tmr, TimerMode tm, FaultMode fm)
{
  // Clear the previous pwm
  disable();
  
	// Check for valid inputs
  if(!setOC(oc)) return false;
  if(!setTimer(tmr)) return false;
  if(!setTimerMode(tm)) return false;
  if(!setFaultMode(fm)) return false;
  
  // Enable PWM
  enable();
}

void PWM::disable()
{
  if(valid_) *(OCxCON[oc_]) &= 0x7FFFFFFF;
}

bool PWM::enable()
{
  if(valid_) *(OCxCON[oc_]) |= (1<<31);
  return valid_;
}

bool PWM::setOC(int oc)
{
  if(oc > 0 && oc <= NUM_OC_PORTS) {
    oc_ = oc;
  }
  else return false;
  return true;
}

bool PWM::setTimer(int tmr)
{
  if(valid_) {
    if(tmr == 2 || tmr == 3) {
      timer_ = tmr;
      if(tmr == 2) *(OCxCON[oc_]) &= ~(1<<3);
      else *(OCxCON[oc_]) |= (1<<3);
    }
    else
      return false;
  }
  else return false;
  return true;
}

bool PWM::setTimerMode(TimerMode tm)
{
  if(valid_) {
    if(tm == SINGLE) *(OCxCON[oc_]) &= ~(1<<5);
    else if(tm == DOUBLE) *(OCxCON[oc_]) |= (1<<5);
  }
  else return false;
  return true;
}

bool PWM::setFaultMode(FaultMode fm)
{
  if(valid_) {
    if(fm == ENABLED) *(OCxCON[oc_]) |= (0b111<<0);
    else {
      *(OCxCON[oc_]) |= (0b11<<1);
      *(OCxCON[oc_]) &= ~(1<<0);
    }
  }
  else return false;
  return true;
}

void PWM::enableTimer(int timer)
{
  if(timer == 2 || timer == 3) {
    *(TxCON[timer-1]) |= (1<<15);
  }
}

void PWM::disableTimer(int timer)
{
  if(timer == 2 || timer == 3) {
    *(TxCON[timer-1]) &= ~(1<<15);
  }
}

float PWM::setTimerFrequency(int timer, float freq, TimerMode timer_mode)
{
  if(timer == 2 || timer == 3) {
    // Need to check if feasible
    unsigned int bit_shift = 16;
    if(freq > MAX_TIMER_FREQ || freq < MIN_TIMER_FREQ) return 0.0;
    if(timer_mode == DOUBLE) bit_shift = 32;
    float est_prescalar = ((float)CLOCK_FREQ)/(freq*(1<<bit_shift));
    if(est_prescalar <= 0.5) est_prescalar = 0.5;
    int prescalar = ceil(log(est_prescalar)/log(2)+1); // Prescalar bits
    if(prescalar < 0) prescalar = 0;
    
    unsigned int counts = ((float)CLOCK_FREQ)/(freq*(1<<prescalar)) - 1;
    unsigned int t_buffer = *(TxCON[timer-1]);
    t_buffer &= ~(0b111<<4);
    t_buffer |= (prescalar<<4);
    *(TxCON[timer-1]) = t_buffer;
    *(PRx[timer-1]) = counts;
    return ((float)CLOCK_FREQ)/((1<<prescalar)*(counts+1));
  }
  else return 0.0;
}

bool PWM::setDuty(float duty)
{
  if(valid_ && duty >= 0.0 && duty <= 1.0) {
    *(OCxRS[timer_]) = duty*(*(PRx[timer_]));
    duty_ = duty;
  }
  else return false;
  return true;
}

bool Motor::reset(int s_oc, int d_pin, int tmr) 
{
	if(valid_ = speed_pwm_.reset(s_oc, tmr)) return false;
  speed_oc_ = s_oc;
	direction_pin_ = d_pin;
}

} // namespace nu_urc_embedded
