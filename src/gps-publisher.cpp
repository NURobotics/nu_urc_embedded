#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS TX (transmit) pin to Digital 3
// Connect the GPS RX (receive) pin to Digital 2
#define mySerial(0, 1); // (RX, TX)
#define GPSECHO true

Adafruit_GPS GPS(&mySerial); 

/* FOR INTERRUPT */
// bool usingInterrupt = false;
// void useInterrupt(bool);

// SIGNAL(TIMER0_COMPA_vect) {
// 	char c = GPS.read();
// #ifdef UDR0
// 	if (GPSECHO)
// 		if (c) UDR0 = c;
// #endif
// }

// void useInterrupt(bool v) {
// 	if (v) {
// 		OCR0 = 0xAF;
// 		TIMSK0 |= _BV(OCIE0A);
// 		usingInterrupt = true;
// 	} else {
// 		TIMSK0 &= ~_BV(OCIE0A);
// 		usingInterrupt = false;
// 	}
// }

// unit32_t timer = millis();

int main(int argc, char **argv) {

	ros::init(argc,argv,"gps-pub");
	ros::NodeHandle n;
	ros::Publisher gps_pub = n.advertise<std_msgs::NavSatFix>();
	ros::Rate loop_rate(10);
	std_msgs::NavSatFix gpsData;

	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PGCMD_ANTENNA);

	//useInterrupt(true);
	
	while (ros::ok()) {
		char c = GPS.read();
		if GPS.newNMEAreceived()) {
			if (!GPS.parse(GPS.lastNMEA()))
				return;
		}

		if (GPS.fix) {
			gpsData.latitudeDegrees = GPS.latitude;
			gpsData.longitudeDegrees = GPS.longitudeDegrees;
			gps_pub.publish(gpsData);
		}

		/* FOR INTERRUPUT */
		// if (! usingInterrupt)
		// 	char c = GPS.read();
		// if (GPSECHO)
		// 	if (c) Serial.print(c);

		// if (GPS.newNMEAreceived()) {
		// 	if (!GPS.parse(GPS.lastNMEA()))
		// 		return;
		// }

		// if (timer > millis()) timer = millis();

		// if (millis() - timer > 2000) {
		// 	if (GPS.fix) {
		// 		timer = millis();
		// 		gpsData.latitudeDegrees = GPS.latitude;
		// 		gpsData.longitudeDegrees = GPS.longitude;
		// 		gps_pub.publish(gpsData);
		// 	}
		// }
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}