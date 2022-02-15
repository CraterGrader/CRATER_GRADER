#include <micro_ros_arduino.h>

#include <stdio.h>
#include <string>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
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

//#define BYTE_TO_QPPS_SPD_SCALE 100
#define BYTE_TO_QPPS_SPD_SCALE 25  // Reduce max speed from what was calibrated
#define BYTE_TO_QPPS_POS_SCALE 21
#define BYTE_TO_QPPS_ZERO_OFFSET 127

#define POSN_CTRL_ACCEL_QPPS 600
#define POSN_CTRL_DECCEL_QPPS 600
#define POSN_CTRL_SPD_QPPS 1300

#define TOOL_CTRL_ACCEL_QPPS 600
#define TOOL_CTRL_DECCEL_QPPS 600
#define TOOL_CTRL_SPD_QPPS 1300

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Instantiate publishers and subscribers
rcl_subscription_t cmd_sub;
rcl_publisher_t debug_msg_pub;

std_msgs__msg__Int64 cmd_msg;
std_msgs__msg__String debug_msg;
bool cmd_msg_received = false;

/* RoboClaws */
#define ROBOCLAW_ADDRESS 0x80
#define NUM_ROBOCLAWS_MOBILITY 2
#define NUM_ROBOCLAWS_TOOL 1
RoboClaw roboclawsmobility[] = {
  RoboClaw(&Serial1,10000), // Pins 18(Tx) and 18(Rx) on the Due
  RoboClaw(&Serial2,10000) // Pins 16(Tx) and 17(Rx) on the Due
};
RoboClaw roboclawstool[] = {
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
      int steer_cmd = byte_to_qpps(steer_cmd_raw, BYTE_TO_QPPS_POS_SCALE, BYTE_TO_QPPS_ZERO_OFFSET);

      int tool_cmd_raw = (cmd_msg.data >> 16) & 0xFF; // Third byte indicates tool height command in range [0, 255]
      if (abs(tool_cmd_raw) <= 1) tool_cmd_raw = 0;  // Fix to zero when close to zero
      int tool_cmd = byte_to_qpps(tool_cmd_raw, BYTE_TO_QPPS_POS_SCALE, 0);

      // Send Mobility Commands
      for (int i = 0; i < NUM_ROBOCLAWS_MOBILITY; ++i) {
        roboclawsmobility[i].SpeedM1(ROBOCLAW_ADDRESS, roboclaw_signs[i]*drive_cmd);
        roboclawsmobility[i].SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, POSN_CTRL_ACCEL_QPPS, POSN_CTRL_SPD_QPPS, POSN_CTRL_DECCEL_QPPS, roboclaw_signs[i]*steer_cmd, 1);
      }

      // Send Tool Commands
      for (int i = 0; i < NUM_ROBOCLAWS_TOOL; i++) {
        roboclawstool[i].SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, TOOL_CTRL_ACCEL_QPPS, TOOL_CTRL_SPD_QPPS, TOOL_CTRL_DECCEL_QPPS, tool_cmd, 1);
      }

      String debug_str = "Currently commanding: {Drive: " + 
        String(drive_cmd) + 
        "; Steer: " + 
        String(steer_cmd) + 
        "; Tool: " + 
        String(tool_cmd_raw) + 
        "}";
        
      debug_msg.data.data = const_cast<char*>(debug_str.c_str());
    } else {
      debug_msg.data.data = const_cast<char*>("No data received");
    }
    RCSOFTCHECK(rcl_publish(&debug_msg_pub, &debug_msg, NULL));
  }
}

// Converts a byte [0, 255] to the appropriate QPPS value
int byte_to_qpps(const int &val, const int &scale, const int &zero_offset)
{
  return (val - zero_offset) * scale;
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
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
  for (auto & roboclaw : roboclawsmobility) {
    roboclaw.begin(38400);
  }
  for (auto & roboclaw : roboclawstool) {
    roboclaw.begin(38400);
  }


}

void loop() {
  delay(LOOP_PERIOD_MS);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(LOOP_PERIOD_MS)));
}
