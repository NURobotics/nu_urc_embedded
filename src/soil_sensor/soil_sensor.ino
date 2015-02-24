#include <SHT1x.h>
#include <ros.h>
#include <soil_msgs/Soil.h>

// Define SDA & SCL
// Uno or Teensy: A4 (SDA), A5 (SLC)
#define dataPin    A4
#define clockPin   A5
SHT1x sht1x(dataPin, clockPin);

// Wiring:
// yellow   - SCK
// red      - VDD (3-5.5V)
// green    - GND
// blue     - DATA


ros::NodeHandle nh;


soil_msgs::Soil soil_msg;
ros::Publish soil_sensor("soil_sensor", &str_msg);

void setup()
{
  nh.initNode();
  nh.advertise(soil_sensor);
  
// SERIAL DEBUGGING  
//  Serial.begin(38400);

}

void loop()
{
  float temp_c;
  float temp_f;
  float humidity;
  
  temp_c = sht1x.readTemperatureC();
  temp_f = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();
  
  soil_msg.temp_c = temp_c;
  soil_msg.temp_f = temp_f;
  soil_msg.humid = humidity;
  nh.spinOnce());
  
// SERIAL DEBUGGING  
//  Serial.print("Temperature: ");
//  Serial.print(temp_c, DEC);
//  Serial.print("C / ");
//  Serial.print(temp_f, DEC);
//  Serial.print("F. Humidity: ");
//  Serial.print(humidity);
//  Serial.println("%");

  delay(1000);
}
