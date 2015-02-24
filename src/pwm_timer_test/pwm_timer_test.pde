#include <ros.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sys/attribs.h>
#include <RoverDriveState.h>

using namespace nu_urc_embedded;

/*ros::NodeHandle nh
sensor_msgs::JointState js;
ros::Publisher p_test("/joints/state", &js);*/

// Motor Init
char* j_name[4] = {"FL_WHEEL", "FR_WHEEL", "BL_WHEEL", "BR_WHEEL"};
float j_position[4];
float j_velocity[4];
float j_effort[4];
float velocity_goals[4];
volatile int count = 0;
float freq = 0;

extern "C" {

void __ISR(_TIMER_2_VECTOR, IPL7SRS)
TimerTest(void)
{
  count++;
  IFS0bits.T2IF = 0;
}
  
}

void setup() 
{ 
  Serial.begin(9600);
  PWM::setTimerFrequency(2,1000);
  PWM::enableTimer(2);
  
  noInterrupts();
  
  IPC2bits.T2IP = 7;
  IPC2bits.T2IS = 3;
  IFS0bits.T2IF = 0;
  IEC0bits.T2IE = 1;
    
  interrupts();
}


void loop() 
{
  Serial.println(count);
  delay(1000);
/*  p_test.publish(&js);
  nh.spinOnce();
  delay(1);*/
}
