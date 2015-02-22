#include <ArduinoHardware.h>
#include <ros.h>
#include <proto_rover_msg/proto_cmds.h>
#include <RoverControl.h>

RoverControl r(3,12,11,13);
ros::NodeHandle nh;
void dir(const proto_rover_msg::proto_cmds& rc)
{
  r.setdirection(rc.x, rc.y);
  if (rc.stop) {
    r.stop();
  }
}
ros::Subscriber<proto_rover_msg::proto_cmds> sub("proto_rover_cmd", &dir);
void setup() 
{
  nh.initNode();
  nh.subscribe(sub);
}
void loop() 
{
  nh.spinOnce();
  delay(1);
}



