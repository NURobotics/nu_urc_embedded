#include <ros.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;
sensor_msgs::JointState js;
ros::Publisher p_test("/joints/state", &js);
char* name[4] = {"FL_WHEEL", "FR_WHEEL", "BL_WHEEL", "BR_WHEEL"};
float position[4];
float velocity[4];
float effort[4];

void setup()
{
  js.name_length = 4;
  js.position_length = 4;
  js.velocity_length = 4;
  js.effort_length = 4;
  js.name = name; // name_length, position_length, velocity_length, effort_length
  js.position = position;
  js.velocity = velocity;
  js.effort = effort;
  
  nh.initNode();
  nh.advertise(p_test);
}

void loop()
{
  p_test.publish(&js); 
  nh.spinOnce();
  delay(100);  
}
