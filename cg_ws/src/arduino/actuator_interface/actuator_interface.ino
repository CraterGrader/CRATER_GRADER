#include <micro_ros_arduino.h>

#include <stdio.h>
#include <string>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <cmath>

#include <std_msgs/msg/int64.h>

#include "RoboClaw.h"

// NUM_HANDLES must be updated to reflect total number of subscribers + publishers
#define NUM_HANDLES 2

#define LED_PIN 13
#define LOOP_PERIOD_MS 10
// TIMER_TIMEOUT_MS appears to control the publishing period
#define TIMER_TIMEOUT_MS 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define BYTE_TO_QPPS_SPD_SCALE 25  // Reduce max speed from what was calibrated
#define BYTE_TO_QPPS_POS_SCALE 21
#define BYTE_TO_QP_POS_SCALE 21
#define BYTE_TO_QPPS_ZERO_OFFSET 127
#define BYTE_TO_QP_TOOL_SCALE -588
#define QP_TO_BYTE_STEER_SCALE 22
#define QP_TO_BYTE_STEER_OFFSET 127
#define QP_TO_BYTE_DRIVE_SCALE 25 // DOUBLE CHECK VALUE FROM LIMITING
#define QP_TO_BYTE_DRIVE_OFFSET 127 
#define QP_TO_BYTE_TOOL_SCALE 22
#define QP_TO_BYTE_TOOL_OFFSET 588 // DOUBLE CHECK VALUE FROM LIMITING


#define POSN_CTRL_ACCEL_QPPS 600
#define POSN_CTRL_DECCEL_QPPS 600
#define POSN_CTRL_SPD_QPPS 1300

#define TOOL_CTRL_ACCEL_QPPS 7000
#define TOOL_CTRL_DECCEL_QPPS 5000
#define TOOL_CTRL_SPD_QPPS 10000

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//Used for validating data stream from roboclaws to Due 
uint8_t status1;
bool valid1;
uint8_t status2;
bool valid2;
uint8_t status3;
bool valid3;
uint8_t status4;
bool valid4;
uint8_t status5;
bool valid5;

// Instantiate publishers and subscribers
rcl_subscription_t cmd_sub;
rcl_publisher_t debug_msg_pub;

std_msgs__msg__Int64 cmd_msg;
std_msgs__msg__Int64 debug_msg;
int64_t iter = 0;

bool cmd_msg_received = false;

/* RoboClaws */
#define ROBOCLAW_ADDRESS 0x80
#define NUM_ROBOCLAWS_MOBILITY 2
#define NUM_ROBOCLAWS_TOOL 1
RoboClaw roboclaws_mobility[] = {
  RoboClaw(&Serial1,10000), // Pins 18(Tx) and 19(Rx) on the Due
  RoboClaw(&Serial2,10000) // Pins 16(Tx) and 17(Rx) on the Due
};
RoboClaw roboclaws_tool[] = {
  RoboClaw(&Serial3,10000) // Pins 14(Tx) and 15(Rx) on the Due
};
// Rear RoboClaw has sign flipped
int roboclaw_signs[] = {
  -1, 1
};

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void cmd_callback(const void * msgin)
{
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
  cmd_msg = *msg;
  cmd_msg_received = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (cmd_msg_received) {
      // Command RoboClaws
      int drive_cmd_raw = cmd_msg.data & 0xFF; // First byte indicates drive command in range [0, 255]
      if (abs(drive_cmd_raw - 127) <= 1) drive_cmd_raw = 127;  // Fix to zero when close to zero
      int drive_cmd = byte_to_qpps(drive_cmd_raw, BYTE_TO_QPPS_SPD_SCALE, BYTE_TO_QPPS_ZERO_OFFSET); 

      int steer_cmd_raw = (cmd_msg.data >> 8) & 0xFF; // Second byte indicates steer command in range [0, 255]
      if (abs(steer_cmd_raw - 127) <= 1) steer_cmd_raw = 127;  // Fix to zero when close to zero
      int steer_cmd = byte_to_qpps(steer_cmd_raw, BYTE_TO_QP_POS_SCALE, BYTE_TO_QPPS_ZERO_OFFSET);

      int tool_cmd_raw = (cmd_msg.data >> 16) & 0xFF; // Third byte indicates tool height command in range [0, 255]
      if (abs(tool_cmd_raw) <= 1) tool_cmd_raw = 0;  // Fix to zero when close to zero
      int tool_cmd = byte_to_qpps(tool_cmd_raw, BYTE_TO_QP_TOOL_SCALE, 0);

      // Send Mobility Commands
      for (int i = 0; i < NUM_ROBOCLAWS_MOBILITY; ++i) {
        roboclaws_mobility[i].SpeedM1(ROBOCLAW_ADDRESS, roboclaw_signs[i]*drive_cmd);
        roboclaws_mobility[i].SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, POSN_CTRL_ACCEL_QPPS, POSN_CTRL_SPD_QPPS, POSN_CTRL_DECCEL_QPPS, roboclaw_signs[i]*steer_cmd, 1);
      }

      // Send Tool Commands
      for (int i = 0; i < NUM_ROBOCLAWS_TOOL; ++i) {
        roboclaws_tool[i].SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, TOOL_CTRL_ACCEL_QPPS, TOOL_CTRL_SPD_QPPS, TOOL_CTRL_DECCEL_QPPS, tool_cmd, 1);
      }

      // Read Encoder Values
      // Read Steer 1 Encoder Value
      int32_t R1enc2 = roboclaws_mobility[0].ReadEncM2(ROBOCLAW_ADDRESS, &status1, &valid1);
      int8_t R1enc2Scale = int32_to_byte(R1enc2, QP_TO_BYTE_STEER_SCALE, QP_TO_BYTE_STEER_OFFSET);

      // Read Steer 2 Encoder Value 
      int32_t R2enc2 = roboclaws_mobility[1].ReadEncM2(ROBOCLAW_ADDRESS, &status2, &valid2);
      int8_t R2enc2Scale = int32_to_byte(R2enc2, QP_TO_BYTE_STEER_SCALE, QP_TO_BYTE_STEER_OFFSET);

      // Read Tool Encoder Value 
      int32_t R3enc1 = roboclaws_tool[0].ReadEncM1(ROBOCLAW_ADDRESS, &status3, &valid3);
      int8_t R3enc1Scale = int32_to_byte(R3enc1, QP_TO_BYTE_TOOL_SCALE, QP_TO_BYTE_TOOL_OFFSET);

      // Read Speed Values
      // Read Drive 1 Speed
      int32_t R1spd1 = roboclaws_mobility[0].ReadSpeedM1(ROBOCLAW_ADDRESS, &status4, &valid4);
      int8_t R1spd1Scale = int32_to_byte(R1spd1, QP_TO_BYTE_DRIVE_SCALE, QP_TO_BYTE_DRIVE_OFFSET);

      // Read Drive 2 Speed 
      int32_t R2spd1 = roboclaws_mobility[1].ReadSpeedM1(ROBOCLAW_ADDRESS, &status5, &valid5);
      int8_t R2spd1Scale = int32_to_byte(R2spd1, QP_TO_BYTE_DRIVE_SCALE, QP_TO_BYTE_DRIVE_OFFSET);

      // Drive delta position front
      int8_t drive_delta_pos_front = 0;

      // Drive delta position rear
      int8_t drive_delta_pos_rear = 0;

      // Terminal byte (limit switches, heartbeat, etc.)
      int8_t term_byte = 0;
  
      // [Steer 1 Encoder, Steer 2 Encoder, Tool Encoder Value, Drive 1 Speed, Drive 2 Speed, Drive Delta Pos Front, Drive Delta Pos Rear, Term Byte]
      debug_msg.data = (term_byte << 56) | (drive_delta_pos_rear << 48) |(drive_delta_pos_front << 40) | (R2spd1Scale << 32) | (R1spd1Scale << 24) | (R3enc1Scale << 16) | (R2enc2Scale << 8) | R1enc2Scale;
      
    } else {
      debug_msg.data = 666;
    }
    RCSOFTCHECK(rcl_publish(&debug_msg_pub, &debug_msg, NULL));
  }
}

// Converts a byte [0, 255] to the appropriate QPPS value
int byte_to_qpps(const int &val, const int &scale, const int &zero_offset)
{
  return (val - zero_offset) * scale;
}

// Converts a int32 to byte [0, 255] 
int8_t int32_to_byte(const int32_t &val, const int &scale, const int &zero_offset)
{
  int raw_value = std::abs((val/scale)+zero_offset); // Get raw value, then clamp 
  return std::min(std::max(raw_value, 0), 255);
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000); // This delay is residual from microros example code. Not sure if it is necessary or not.

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "arduino_actuator_interface_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "arduino_cmd"));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &debug_msg_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "arduino_debug"));

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
    timer_callback));

  cmd_msg.data = 0;

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, NUM_HANDLES, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA));

  // Set up RoboClaws
  for (auto & roboclaw : roboclaws_mobility) {
    roboclaw.begin(38400);
  }
  for (auto & roboclaw : roboclaws_tool) {
    roboclaw.begin(38400);
  }


}

void loop() {
  delay(LOOP_PERIOD_MS);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(LOOP_PERIOD_MS)));
}
