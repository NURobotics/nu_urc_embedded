// Motor Control Test

#include <ros.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sys/attribs.h>

#define NUM_DRIVE_JOINTS    4

enum {
  FL_WHEEL = 0,
  FR_WHEEL,
  BL_WHEEL,
  BR_WHEEL  
};

void velocityJointCommandsCallback(const nu_urc_msgs::JointCommands&);

nu_urc_msgs::JointCommands jc_buffer;
ros::NodeHandle nh;
sensor_msgs::JointState js;
ros::Publisher js_publisher("/joints/state", &js);
ros::Publisher jc_pub_test("/test/joints/cmd", &jc_buffer);
ros::Subscriber<nu_urc_msgs::JointCommands> jc_subscriber("/nu_urc_rover/joints/cmds", &velocityJointCommandsCallback);
char* j_name[4] = {"FL_WHEEL", "FR_WHEEL", "BL_WHEEL", "BR_WHEEL"};
float j_position[4];
float j_velocity[4];
float j_effort[4];
float velocity_goals[4];
int count = 0;

extern "C" {

void __ISR(_TIMER_4_VECTOR, IPL7SRS)
TimerTest(void)
{
  count++;
  IFS0bits.T4IF  =0;
}
  
void __ISR(_CHANGE_NOTICE_VECTOR, IPL7SRS) 
EncoderUpdate(void)
{ 
  IFS1bits.CNIF = 0;                    // Clear Interrupt Flag
}

}

void setup()
{
  js.name_length = js.position_length = js.velocity_length = js.effort_length = 4;
  js.name = j_name;
  js.position = j_position;
  js.velocity = j_velocity;
  js.effort = j_effort;
  
  jc_buffer.names_length = 4;
  jc_buffer.commands_length = 4;
  jc_buffer.names = j_name;
  jc_buffer.commands = velocity_goals;

  // Timer Setup
  T4CONbits.ON = 0;
  T4CONbits.TCKPS = 4;
  T4CONbits.TCS = 0;
  T4CONbits.TGATE = 0;
  TMR4 = 0;
  PR4 = 49999;
  
  noInterrupts();
  
  IPC4bits.T4IP = 7;
  IPC4bits.T4IS = 3;
  IFS0bits.T4IF = 0;
  IEC0bits.T4IE = 1;
  
  interrupts();
  
  nh.initNode();
  nh.advertise(js_publisher);
  nh.advertise(jc_pub_test);
  nh.subscribe(jc_subscriber);

  jc_buffer.commands[0] = 100.0;  
  jc_buffer.commands[1] = 100.0;
  jc_buffer.commands[2] = 100.0;
  jc_buffer.commands[3] = 100.0;
}


void loop()
{
  js_publisher.publish(&js);
  jc_pub_test.publish(&jc_buffer);
  nh.spinOnce();
  delay(100);  
}

void velocityJointCommandsCallback(const nu_urc_msgs::JointCommands& j_cmds)
{
  for(int i = 0; i < j_cmds.names_length; i++) {
    if(!strcmp(j_cmds.names[i],"front_left_wheel")) velocity_goals[0] = j_cmds.commands[i];
    else if(!strcmp(j_cmds.names[i],"front_right_wheel")) velocity_goals[1] = j_cmds.commands[i];
    else if(!strcmp(j_cmds.names[i],"back_left_wheel")) velocity_goals[2] = j_cmds.commands[i];
    else if(!strcmp(j_cmds.names[i],"back_right_wheel")) velocity_goals[3] = j_cmds.commands[i];
  }
}
