//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
//SoftwareSerial serial(10,11);	
//RoboClaw roboclaw(&serial,10000);

constexpr int NUM_ROBOCLAWS = 2;
constexpr int NUM_MOTORS = NUM_ROBOCLAWS*2;
SoftwareSerial serials[] = {
  SoftwareSerial(8,9),
  SoftwareSerial(10,11)
};
RoboClaw roboclaws[] = {
  RoboClaw(&serials[0],10000),
  RoboClaw(&serials[1],10000)
};

#define address 0x80
int fwdbkwd_spd = 64; // initialize to zero speed

constexpr uint8_t ZERO_SPD = 64;
uint8_t serial_buf[NUM_MOTORS]; // buffer for reading serial commands

void setup() {
  //Open roboclaw serial ports
  for (auto & roboclaw : roboclaws) {
    roboclaw.begin(38400);  
  }
  Serial.begin(115200);
  Serial.setTimeout(1);

  for (int i = 0; i < NUM_MOTORS; ++i) {
    serial_buf[i] = ZERO_SPD;
  }
}

void loop() {
  for (int i = 0; i < NUM_ROBOCLAWS; ++i) {
    roboclaws[i].ForwardBackwardM1(address,serial_buf[i*2]); // control Motor1 on [0,127] where 0 = backward full speed, 127 = forward full speed, 64 = zero speed
    roboclaws[i].ForwardBackwardM2(address,serial_buf[i*2+1]); // control Motor2 on [0,127] where 0 = backward full speed, 127 = forward full speed, 64 = zero speed
  }
  while (!Serial.available());
  Serial.readBytes(serial_buf, NUM_MOTORS);
  for (int i = 0; i < NUM_MOTORS; ++i) {
    serial_buf[i] &= 0x7F;
  }
  
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
