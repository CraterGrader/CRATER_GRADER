//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(8,9);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

//Velocity PID coefficients.
#define Kp 0.25 // 1.654
#define Ki 0.01 // 0.165
#define Kd 0 // 0
#define qpps 8779 // 10312

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(57600);
  roboclaw.begin(38400);
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kp,Ki,Kd,qpps);
  roboclaw.SetM2VelocityPID(address,Kp,Ki,Kd,qpps);
//  roboclaw.SetM2PositionPID(address,Kd,Kp,Ki,1,0,0,qpps);
}

void displayspeed(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  int32_t ispeed2 = roboclaw.ReadISpeedM2(address);
  float kp_vel, ki_vel, kd_vel;
  uint32_t qpps_vel;
  roboclaw.ReadM2VelocityPID(address, kp_vel, ki_vel, kd_vel, qpps_vel);
  // TODO don't use floating point comparisons
  if (Kp != kp_vel || Ki != ki_vel || Kd != kd_vel || qpps_vel != qpps) {
    roboclaw.SetM2VelocityPID(address,Kp,Ki,Kd,qpps);  
  }

  Serial.print("Encoder1:");
  if(valid1){
    Serial.print(enc1);
    Serial.print(" ");
    Serial.print(status1);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Encoder2:");
  if(valid2){
    Serial.print(enc2);
    Serial.print(" ");
    Serial.print(status2);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Speed1:");
  if(valid3){
    Serial.print(speed1);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Speed2:");
  if(valid4){
    Serial.print(speed2);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print(" iSpeed2:"); Serial.print(ispeed2);
  Serial.print(" kp:"); Serial.print(kp_vel);
  Serial.print(" ki:"); Serial.print(ki_vel);
  Serial.print(" kd:"); Serial.print(kd_vel);
  Serial.print(" qpps:"); Serial.print(qpps_vel);
  Serial.println();
  
}

void loop() {
  roboclaw.SpeedM2(address,1000);
  for(uint8_t i = 0;i<100;i++){
    displayspeed();
    delay(10);
  }
  roboclaw.SpeedM2(address,2000);
  for(uint8_t i = 0;i<100;i++){
    displayspeed();
    delay(10);
  }
  roboclaw.SpeedM2(address,3000);
  for(uint8_t i = 0;i<100;i++){
    displayspeed();
    delay(10);
  }
}
