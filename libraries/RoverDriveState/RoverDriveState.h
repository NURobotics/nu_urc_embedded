#ifndef ROVERDRIVESTATE_H
#define ROVERDRIVESTATE_H

#include <WProgram.h>
#include <xc.h>
#include <sys/attribs.h>
#include <math.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>

#define NUM_DRIVE_JOINTS    4
#define NUM_OC_PORTS        5
#define NUM_TIMERS          5
#define COUNT_TO_RAD        1.0
#define CLOCK_FREQ          80000000
#define MAX_TIMER_FREQ      80000000.0
#define MIN_TIMER_FREQ      4.8

namespace nu_urc_embedded {

extern volatile unsigned int* OCxCON[NUM_OC_PORTS];
extern volatile unsigned int* OCxRS[NUM_OC_PORTS];
extern volatile unsigned int* OCxR[NUM_OC_PORTS];
extern volatile unsigned int* TxCON[NUM_OC_PORTS];
extern volatile unsigned int* TMRx[NUM_OC_PORTS];
extern volatile unsigned int* PRx[NUM_OC_PORTS];

enum {
  FL_WHEEL,
  FR_WHEEL,
  BL_WHEEL,
  BR_WHEEL
};

enum TimerMode {
  SINGLE,
  DOUBLE
};

enum FaultMode {
  ENABLED,
  DISABLED
};

class PWM
{
public:
  PWM() { valid_ = false; }
	PWM(int oc, int tmr=2, TimerMode tm=SINGLE, FaultMode fm=DISABLED) 
  { 
    valid_ = reset(oc, tmr, tm, fm); 
  }
  
  bool valid() { return valid_; }
	bool reset(int oc, int tmr=2, TimerMode=SINGLE, FaultMode fm=DISABLED);
  bool setTimer(int tmr);
  bool setTimerMode(TimerMode tm);
  bool setFaultMode(FaultMode fm);
  bool setDuty(float duty);
  bool setOC(int oc);
  float getDuty() { return duty_; }
  bool enable();
  void disable();

  static float setTimerFrequency(int timer, float freq, TimerMode timer_mode=SINGLE);
  static void enableTimer(int timer);
  static void disableTimer(int timer);
private:
  FaultMode fault_mode_;
  TimerMode timer_mode_;
  int timer_;
	int oc_;
  bool valid_;
  float duty_;
  float frequency_;
};

class Motor
{
public:
	Motor() { valid_ = false; }
	Motor(int s_oc, int d_pin, int tmr) { reset(s_oc, d_pin, tmr); }
  
  bool valid() { return valid_; }
	bool reset(int s_oc, int d_pin, int tmr=2);
  bool setDuty(float duty) { return speed_pwm_.setDuty(duty); }
  float getDuty() { return speed_pwm_.getDuty(); };
  bool setDirection(int dir) { digitalWrite(direction_pin_, dir); }
  bool getDirection() { return digitalRead(direction_pin_); }
	PWM speed_pwm_;
private:
	int speed_oc_;
	int direction_pin_;
  bool valid_;
};

class Encoder
{
public:
  Encoder() {}
	Encoder(int A_pin, int B_pin) { A_pin_ = A_pin; B_pin_ = B_pin; }
	float getAngle() { return count_*COUNT_TO_RAD; };
	int count_;
  int A_pin_;
  int B_pin_;
  int A_buffer_;
  int B_buffer_;
};

class JointControl
{
public:
	Motor motor_;
	Encoder encoder_;
  volatile double velocity_goal_;
  volatile double position_;
  volatile double velocity_;
  volatile double effort_;
};

class RoverDriveState
{
public:
	RoverDriveState() {}
	void reset() {}
  sensor_msgs::JointState js_;
	JointControl joint_control_[NUM_DRIVE_JOINTS];
};

} // namespace nu_urc_embedded

#endif // ROVERDRIVESTATE_H
