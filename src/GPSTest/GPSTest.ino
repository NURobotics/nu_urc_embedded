#include <WProgram.h>
#include <Teensy_GPS.h>
#include <ros.h>
#include <ultimate_gps/GpsFix.h>

ros::NodeHandle nh;

ultimate_gps::GpsFix gps_data;
ros::Publisher gps_pub("gps_pub", &gps_data);

HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial,true);

// setup() method runs once, when the sketch starts
void setup()
{
Serial.begin(38400); // For debugging output over the USB port
gps.startSerial(9600);
delay(1000);
gps.setSentencesToReceive(OUTPUT_RMC_GGA);
nh.initNode();
nh.advertise(gps_pub);
}

bool gotGPS = false;
// the loop() methor runs over and over again,
// as long as the board has power
void loop()
{
if (gps.sentenceAvailable()) gps.parseSentence();

if (gps.newValuesSinceDataRead()) {
gotGPS = true;
gps.dataRead();
Serial.printf("Location: %f, %f altitude %f\n\r",
gps.latitude, gps.longitude, gps.altitude);
gps_data.latitude = gps.latitude;
gps_data.longitude = gps.longitude;
gps_pub.publish(&gps_data);
nh.spinOnce();
delay(500);
}
//else {
//  Serial.printf("No GPS read\n");
//}
}
// Called from the powerup interrupt servicing routine.
int main(void)
{
setup();
while (true) {
loop();
yield();
}
return 0; // Never reached.
}
