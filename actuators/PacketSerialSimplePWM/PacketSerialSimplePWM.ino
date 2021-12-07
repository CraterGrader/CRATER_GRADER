//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80
int fwdbkwd_spd = 64; // initialize to zero speed

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  roboclaw.ForwardBackwardM1(address,fwdbkwd_spd); // control Motor1 on [0,127] where 0 = backward full speed, 127 = forward full speed, 64 = zero speed
//  roboclaw.ForwardBackwardM2(address,fwdbkwd_spd); // control Motor2 on [0,127] where 0 = backward full speed, 127 = forward full speed, 64 = zero speed
  while (!Serial.available());
  fwdbkwd_spd = Serial.readString().toInt();
  
  /*
  roboclaw.BackwardM1(address,64);
  roboclaw.ForwardM2(address,64);
  delay(2000);

  roboclaw.ForwardBackwardM1(address,96); //start Motor1 forward at half speed
  roboclaw.ForwardBackwardM2(address,32); //start Motor2 backward at half speed
  delay(2000);

  roboclaw.ForwardBackwardM1(address,32);
  roboclaw.ForwardBackwardM2(address,96);
  delay(2000);
  */
}
