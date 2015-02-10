#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>

//Insert correct pin for serial
#define mySerial 9

Adafruit_GPS GPS(&mySerial); 

int main(int argc, char **argv) {

	ros::init(argc,argv,"gps-pub");
	ros::NodeHandle n;
	ros::Publisher gps_pub = n.advertise<std_msgs::NavSatFix>();
	ros::Rate loop_rate(10);
	
	
	while (ros::ok()) {
		GPS.begin(9600);
		
		std_msgs::NavSatFix gpsData;
		
		gpsData.latitude = GPS.latitude;
		gpsData.longitude = GPS.longitude;
		gps_pub.publish(gpsData);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

