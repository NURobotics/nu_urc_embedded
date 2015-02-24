// Change Notification Test

#include <ros.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>
#include <nu_urc_msgs/JointCommands.h>
#include <sys/attribs.h>
#include <RoverDriveState.h>

int counter = 0;

extern "C" {

void __ISR(_CHANGE_NOTICE_VECTOR, IPL7SRS)
ChangeNoticeTest(void)
{
  counter++;
  IFS1bits.CNIF = 0;
}
  
}

void setup()
{
  Serial.begin(9600);
  
  // Configure bits 15,16,17,18
  CNCONbits.ON = 0;
  
  CNPUEbits.CNPUE15= 1;
  CNPUEbits.CNPUE16 = 1;
  CNPUEbits.CNPUE17 = 1;
  CNPUEbits.CNPUE18 = 1;
  
  CNENbits.CNEN15 = 1;
  CNENbits.CNEN16 = 1;
  CNENbits.CNEN17 = 1;
  CNENbits.CNEN18 = 1;
  
  noInterrupts();
  
  IPC6bits.CNIP = 7;
  IPC6bits.CNIS = 3;
  IFS1bits.CNIF = 0;
  IEC1bits.CNIE = 1;
  
  interrupts();
  
  CNCONbits.ON = 1;
}

void loop()
{
  Serial.println(counter);  
  delay(1000);
}
