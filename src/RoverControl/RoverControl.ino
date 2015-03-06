#include <ArduinoHardware.h>
#include <ros.h>
#include <proto_rover_msg/proto_cmd.h>
#include <RoverControl.h>

RoverControl r(3,12,11,13);
ros::NodeHandle n;
void dir(const proto_rover_msg::proto_cmd& rc)
{
  if (rc.stop) {
    r.stop();
  }
  else {
  r.setdirection(rc.y, rc.x);
  }
}
ros::Subscriber<proto_rover_msg::proto_cmd> sub("proto_rover_cmds", dir);
void setup() 
{
  n.initNode();
  n.subscribe(sub);
}
void loop() 
{
  n.spinOnce();
  delay(1);
}
