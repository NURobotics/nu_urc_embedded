#include <SoftwareSerial.h>

#include <Adafruit_GPS.h>

#include <ArduinoHardware.h>
#include <ros.h>

#include <gps_msg/coordinates.h>

SoftwareSerial s(3,2);

Adafruit_GPS gps(&s);

ros::NodeHandle n;
gps_msg::coordinates pos;
ros::Publisher pub("gpspos", &pos);

void setup() 
{
  Serial.begin(115200);
  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  gps.sendCommand(PGCMD_ANTENNA);
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  n.initNode();
  n.advertise(pub);
}

SIGNAL(TIMER0_COMPA_vect) {
  char g = gps.read();
}
void loop () 
{
  pos.fix = gps.fix;
  pos.satellites = gps.satellites;
  if (gps.fix) {
    pos.latitude = gps.latitude;
    pos.longitude = gps.longitude;
    pos.altitude = gps.altitude;
    pos.knots = gps.speed;
  }
  pub.publish(&pos);
  n.spinOnce();
  delay(500);
}
