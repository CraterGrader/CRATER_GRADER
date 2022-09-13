//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

//Display Encoder and Speed for Motor 2
void displayspeed(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc2 = roboclaw.ReadEncM2(address, &status1, &valid1);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status2, &valid2);
  
  if(valid1){
    Serial.print("Encoder2:");
    Serial.print(enc2,DEC);
    Serial.print(" ");
    Serial.print(status2,HEX);
    Serial.print(" ");
  }
  if(valid2){
    Serial.print("Speed2:");
    Serial.print(speed2,DEC);
    Serial.print(" ");
  }
  
  Serial.println();
}

//This is the first function arduino runs on reset/power up
void setup() {
  //Open Serial and roboclaw at 38400bps
  Serial.begin(57600);
  roboclaw.begin(38400);
  
  Serial.println("Starting...");
}

void loop() {
  //address, accel, speed, deccel, position, flag
  roboclaw.SpeedAccelDeccelPositionM2(address,100,5000,1000,4800,0);
  roboclaw.SpeedAccelDeccelPositionM2(address,100,5000,1000,-4800,0);
  long last = millis();
  while(millis()-last<5000){
    displayspeed();
    delay(50);
  }
}
