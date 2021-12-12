//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
//SoftwareSerial serial(10,11);	
//RoboClaw roboclaw(&serial,10000);

constexpr int NUM_ROBOCLAWS = 2;
constexpr int NUM_MOTORS = NUM_ROBOCLAWS*2;
constexpr int NUM_BYTES = 5; // [b1m1, b1m2, b2m1, b2m2, estop]

// Indices for byte array
constexpr int B1M1_IDX = 0;
constexpr int B1M2_IDX = 1;
constexpr int B2M1_IDX = 2;
constexpr int B2M2_IDX = 3;
constexpr int ESTOP_IDX = 4;

// Values for motor commands
int b1m1_val;
int b1m2_val;
int b2m1_val;
int b2m2_val;
int estop_val;

// Position control - for use with function roboclaw.SpeedAccelDeccelPositionMX()
constexpr int POS_ACCEL = 100;
constexpr int POS_DECCEL = 100;
constexpr int POS_SPD = 5000;

// Scaling and offset factors
constexpr int SPD_SCALE = 100;
constexpr int POS_SCALE = 194;
constexpr int OFFSET = 127;

// Setup serial ports to RoboClaw controllers
SoftwareSerial serials[] = {
  SoftwareSerial(8,9),
  SoftwareSerial(10,11)
};

// Initialize RoboClaw objects
// [board1, board2]
constexpr int ROBOBOARD1_IDX = 0;
constexpr int ROBOBOARD2_IDX = 1;
RoboClaw roboclaws[] = {
  RoboClaw(&serials[0],10000),
  RoboClaw(&serials[1],10000)
};
#define address 0x80 // stock from RoboClaw code

uint8_t scaled_cmds[NUM_BYTES*2]; // buffer for reading serial commands

void setup() {
  //Open roboclaw serial ports
  for (auto & roboclaw : roboclaws) {
    roboclaw.begin(38400);  
  }
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {

  // Read motor commands from serial interface
  while (!Serial.available());
  Serial.readBytes(scaled_cmds, NUM_BYTES);
//  Serial.println("New message!");
  for (int i = 0; i < NUM_BYTES; ++i) {
//    scaled_cmds[i] = scaled_cmds[i] - '0'; // Subtract for ascii 
    Serial.println(scaled_cmds[i]);
  }

  // Unscale motor commands
  b1m1_val = scale_and_offset(scaled_cmds[B1M1_IDX], SPD_SCALE, OFFSET);
  b1m2_val = scale_and_offset(scaled_cmds[B1M2_IDX], POS_SCALE, OFFSET);
  b2m1_val = scale_and_offset(scaled_cmds[B2M1_IDX], SPD_SCALE, OFFSET);
  b2m2_val = scale_and_offset(scaled_cmds[B2M2_IDX], POS_SCALE, OFFSET);
  estop_val = scaled_cmds[ESTOP_IDX];

  // Print for debugging
  Serial.print("[");
  Serial.print(b1m1_val);
  Serial.print(", ");
  Serial.print(b1m2_val);
  Serial.print(", ");
  Serial.print(b2m1_val);
  Serial.print(", ");
  Serial.print(b2m2_val);
  Serial.print(", ");
  Serial.print(estop_val);
  Serial.print("]");

  // Set motor commands
  // Board1 M1 - Speed control
  roboclaws[ROBOBOARD1_IDX].SpeedM1(address,b1m1_val);
  // Board1 M2 - Position control, set flag=1 so command is followed immediately
  roboclaws[ROBOBOARD1_IDX].SpeedAccelDeccelPositionM2(address,POS_ACCEL,POS_SPD,POS_DECCEL,b1m2_val,1); //address, accel, speed, deccel, position, flag
  // Board2 M1 - Speed control
  roboclaws[ROBOBOARD2_IDX].SpeedM1(address,b2m1_val);
  // Board2 M2 - Position control
  roboclaws[ROBOBOARD2_IDX].SpeedAccelDeccelPositionM2(address,POS_ACCEL,POS_SPD,POS_DECCEL,b2m2_val,1); //address, accel, speed, deccel, position, flag
}

// Undo scaling and offset to get desired value back
int scale_and_offset(int val, int scale, int offset){
  int result;
  result = (val - offset) * scale;
  return result;
}
