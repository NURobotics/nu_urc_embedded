#include <SoftwareSerial.h>

#include <Adafruit_GPS.h>

#include <ArduinoHardware.h>
#define USE_USBCON
#include <ros.h>

#include <gps_msg/coordinates.h>

SoftwareSerial s(3,2);

Adafruit_GPS gps(&s);
char g;
char spg[1];
ros::NodeHandle n;
gps_msg::coordinates pos;
ros::Publisher pub("gpspos", &pos);

void setup() 
{
  //Serial.begin(115200);
  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  gps.sendCommand(PGCMD_ANTENNA);
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  n.initNode();
  n.advertise(pub);
  n.getHardware()->setBaud115200);
}

SIGNAL(TIMER0_COMPA_vect) {
  g = gps.read();
  //pos.knots = 1;
  //n.loginfo("tis");
}
void loop () 
{
  gps.parse(gps.lastNMEA());
  //spg[1] = gps.read();
  //pos.gps = spg;
  pos.fix = gps.fix;
  pos.satellites = gps.satellites;
  pos.latitude = gps.latitude;
  if (gps.fix) {
    pos.latitude = 1;
    pos.longitude = gps.longitude;
    pos.altitude = gps.altitude;
    pos.knots = gps.speed;
  }
  pub.publish(&pos);
  n.spinOnce();
  delay(500);
}
