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

#define ERROR_PIN 13
#define CONNECTED_TO_AGENT_PIN 12
long int last_recv_msg_time; // Time when last message was received from arduino_cmd topic
#define RECV_MSG_CONNECTION_TIMEOUT_SEC 20 // Threshold time for detecting a dis-connection, must be greater than the attachment time (observed to be ~10sec for one publisher, one subscriber, and one timer callback)

#define LOOP_PERIOD_MS 10
// TIMER_TIMEOUT_MS appears to control the publishing period
#define TIMER_TIMEOUT_MS 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define BYTE_TO_QPPS_DRIVE_SCALE 25  // Drive Scale 
#define BYTE_TO_QP_STEER_SCALE 22 // Steer Scale
#define BYTE_TO_QPPS_DRIVE_STEER_OFFSET 127 // 
#define BYTE_TO_QP_DELTA_POS_SCALE 10  // Drive Scale 
#define BYTE_TO_QP_DELTA_POS_OFFSET 127 // 


#define BYTE_TO_QP_TOOL_SCALE -588 // Tool Scale
#define BYTE_TO_QP_TOOL_OFFSET 0 // Tool offset 

#define POSN_CTRL_ACCEL_QPPS 600
#define POSN_CTRL_DECCEL_QPPS 600
#define POSN_CTRL_SPD_QPPS 1300

#define TOOL_CTRL_ACCEL_QPPS 7000
#define TOOL_CTRL_DECCEL_QPPS 5000
#define TOOL_CTRL_SPD_QPPS 10000

rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

//Used for validating data stream from roboclaws to Due
uint8_t rc_0_steer_status; //Steer 1
bool rc_0_steer_valid; //Steer 1
uint8_t rc_1_steer_status; //Steer 2
bool rc_1_steer_valid; //Steer 2
uint8_t rc_2_tool_status; //Tool
bool rc_2_tool_valid; //Tool
uint8_t rc_0_drive_status; //Drive 1
bool rc_0_drive_valid; //Drive 1
uint8_t rc_1_drive_status; //Drive 2
bool rc_1_drive_valid; //Drive 2

// Instantiate publishers and subscribers
rcl_subscription_t cmd_sub;
rcl_publisher_t feedback_pub;

std_msgs__msg__Int64 cmd_msg;
std_msgs__msg__Int64 feedback_msg;

int32_t deltaPosM1Last = 0;
int32_t deltaPosM1Curr = 0;
int32_t deltaPosM2Last = 0;
int32_t deltaPosM2Curr = 0;

bool cmd_msg_received = false;

/* RoboClaws */
#define ROBOCLAW_ADDRESS 0x80
#define NUM_ROBOCLAWS_MOBILITY 2
#define NUM_ROBOCLAWS_TOOL 1
RoboClaw roboclaws_mobility[] = {
  RoboClaw(&Serial1, 10000), // Pins 18(Tx) and 19(Rx) on the Due
  RoboClaw(&Serial2, 10000) // Pins 16(Tx) and 17(Rx) on the Due
};
RoboClaw roboclaws_tool[] = {
  RoboClaw(&Serial3, 10000) // Pins 14(Tx) and 15(Rx) on the Due
};
// Rear RoboClaw has sign flipped
int roboclaw_signs[] = {
  -1, 1
};
rcl_ret_t rc;
void destroy_entities()
{
  // Clear all node artifacts so a new attachment attempt can be made
  rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&feedback_pub, &node);
  rc += rcl_timer_fini(&timer);
  rc += rcl_subscription_fini(&cmd_sub, &node);
  rc += rcl_node_fini(&node);
  rc += rclc_support_fini(&support);

 if (rc != RCL_RET_OK) {
   for (int i = 0; i <= 21; i++) {
      digitalWrite(ERROR_PIN, !digitalRead(ERROR_PIN));
      digitalWrite(CONNECTED_TO_AGENT_PIN, !digitalRead(CONNECTED_TO_AGENT_PIN));
      delay(50);
    }
  }
}

void error_loop() {
    digitalWrite(CONNECTED_TO_AGENT_PIN, LOW); // Turn off agent connection pin
    for (int i = 0; i <= 7; i++) {
      // Error signal
      digitalWrite(ERROR_PIN, !digitalRead(ERROR_PIN));
      delay(100);
    }

    // Destroy entities to start fresh
    destroy_entities();
    delay(100); // Short delay to ensure all entities are fully destroyed before making next attempt
    
    last_recv_msg_time = millis();
    // Try to make connection again
    start_uros_node();
}

void cmd_callback(const void * msgin)
{
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
  cmd_msg = *msg;
  cmd_msg_received = true;
  last_recv_msg_time = millis();
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

  // Check for timeout
  if ((millis() - last_recv_msg_time) / 1000.0 >= RECV_MSG_CONNECTION_TIMEOUT_SEC)
  {
    // Signal for the error and run the error loop
    digitalWrite(ERROR_PIN, HIGH);
    error_loop();
  }

  // Connection is good
  digitalWrite(CONNECTED_TO_AGENT_PIN, HIGH); // Turn on agent connection pin
  digitalWrite(ERROR_PIN, LOW); // Turn on error pin

  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (cmd_msg_received) {
      // Command RoboClaws
      int drive_cmd_raw = cmd_msg.data & 0xFF; // First byte indicates drive command in range [0, 255]
      if (abs(drive_cmd_raw - 127) <= 1) drive_cmd_raw = 127;  // Fix to zero when close to zero
      int drive_cmd = byte_to_qpps(drive_cmd_raw, BYTE_TO_QPPS_DRIVE_SCALE, BYTE_TO_QPPS_DRIVE_STEER_OFFSET);

      int steer_cmd_raw = (cmd_msg.data >> 8) & 0xFF; // Second byte indicates steer command in range [0, 255]
      if (abs(steer_cmd_raw - 127) <= 1) steer_cmd_raw = 127;  // Fix to zero when close to zero
      int steer_cmd = byte_to_qpps(steer_cmd_raw, BYTE_TO_QP_STEER_SCALE, BYTE_TO_QPPS_DRIVE_STEER_OFFSET);

      int tool_cmd_raw = (cmd_msg.data >> 16) & 0xFF; // Third byte indicates tool height command in range [0, 255]
      if (abs(tool_cmd_raw) <= 1) tool_cmd_raw = 0;  // Fix to zero when close to zero
      int tool_cmd = byte_to_qpps(tool_cmd_raw, BYTE_TO_QP_TOOL_SCALE, BYTE_TO_QP_TOOL_OFFSET);

      // Send Mobility Commands
      for (int i = 0; i < NUM_ROBOCLAWS_MOBILITY; ++i) {
        roboclaws_mobility[i].SpeedM1(ROBOCLAW_ADDRESS, roboclaw_signs[i]*drive_cmd);
        roboclaws_mobility[i].SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, POSN_CTRL_ACCEL_QPPS, POSN_CTRL_SPD_QPPS, POSN_CTRL_DECCEL_QPPS, roboclaw_signs[i]*steer_cmd, 1);
      }

      // Send Tool Commands
      for (int i = 0; i < NUM_ROBOCLAWS_TOOL; ++i) {
        roboclaws_tool[i].SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, TOOL_CTRL_ACCEL_QPPS, TOOL_CTRL_SPD_QPPS, TOOL_CTRL_DECCEL_QPPS, tool_cmd, 1);
      }

    } // if (cmd_msg_received)

    // Read Encoder Values
    // Read Steer 1 Encoder Value
    int32_t R1enc2 = roboclaws_mobility[0].ReadEncM2(ROBOCLAW_ADDRESS, &rc_0_steer_status, &rc_0_steer_valid);
    uint8_t R1enc2Scale = int32_to_byte(R1enc2, BYTE_TO_QP_STEER_SCALE, BYTE_TO_QPPS_DRIVE_STEER_OFFSET);

    // Read Steer 2 Encoder Value
    int32_t R2enc2 = roboclaws_mobility[1].ReadEncM2(ROBOCLAW_ADDRESS, &rc_1_steer_status, &rc_1_steer_valid);
    uint8_t R2enc2Scale = int32_to_byte(R2enc2, BYTE_TO_QP_STEER_SCALE, BYTE_TO_QPPS_DRIVE_STEER_OFFSET);

    // Read Tool Encoder Value
    int32_t R3enc1 = roboclaws_tool[0].ReadEncM1(ROBOCLAW_ADDRESS, &rc_2_tool_status, &rc_2_tool_valid);
    uint8_t R3enc1Scale = int32_to_byte(R3enc1, BYTE_TO_QP_TOOL_SCALE, BYTE_TO_QP_TOOL_OFFSET);

    // Read Speed Values
    // Read Drive 1 Speed
    int32_t R1spd1 = roboclaws_mobility[0].ReadSpeedM1(ROBOCLAW_ADDRESS, &rc_0_drive_status, &rc_0_drive_valid);
    uint8_t R1spd1Scale = int32_to_byte(R1spd1, BYTE_TO_QPPS_DRIVE_SCALE, BYTE_TO_QPPS_DRIVE_STEER_OFFSET);

    // Read Drive 2 Speed
    int32_t R2spd1 = roboclaws_mobility[1].ReadSpeedM1(ROBOCLAW_ADDRESS, &rc_1_drive_status, &rc_1_drive_valid);
    uint8_t R2spd1Scale = int32_to_byte(R2spd1, BYTE_TO_QPPS_DRIVE_SCALE, BYTE_TO_QPPS_DRIVE_STEER_OFFSET);

    // Drive delta position front
    deltaPosM1Curr = roboclaws_mobility[0].ReadEncM1(ROBOCLAW_ADDRESS, &rc_0_drive_status, &rc_0_drive_valid);
    uint8_t drive_delta_pos_front = int32_to_byte(deltaPosM1Curr - deltaPosM1Last, BYTE_TO_QP_DELTA_POS_SCALE, BYTE_TO_QP_DELTA_POS_OFFSET);
    deltaPosM1Last = deltaPosM1Curr;

    // Drive delta position rear
    deltaPosM2Curr = roboclaws_mobility[1].ReadEncM1(ROBOCLAW_ADDRESS, &rc_1_drive_status, &rc_1_drive_valid);
    uint8_t drive_delta_pos_rear = int32_to_byte(deltaPosM2Curr - deltaPosM2Last, BYTE_TO_QP_DELTA_POS_SCALE, BYTE_TO_QP_DELTA_POS_OFFSET);
    deltaPosM2Last = deltaPosM2Curr;

    // Terminal byte (limit switches, heartbeat, etc.)
    uint8_t term_byte = 0;

    // [Steer 1 Encoder, Steer 2 Encoder, Tool Encoder Value, Drive 1 Speed, Drive 2 Speed, Drive Delta Pos Front, Drive Delta Pos Rear, Term Byte]
    feedback_msg.data = (static_cast<uint64_t>(term_byte) << 56) | (static_cast<uint64_t>(drive_delta_pos_rear) << 48) | (static_cast<uint64_t>(drive_delta_pos_front) << 40) | (static_cast<uint64_t>(R2spd1Scale) << 32) | (static_cast<uint64_t>(R1spd1Scale) << 24) | (static_cast<uint64_t>(R3enc1Scale) << 16) | (static_cast<uint64_t>(R2enc2Scale) << 8) | static_cast<uint64_t>(R1enc2Scale);

    RCSOFTCHECK(rcl_publish(&feedback_pub, &feedback_msg, NULL));

  } // if (timer != NULL)
}

// Converts a byte [0, 255] to the appropriate QPPS value
int byte_to_qpps(const int &val, const int &scale, const int &zero_offset)
{
  return (val - zero_offset) * scale;
}

// Converts a int32 to byte [0, 255]
int8_t int32_to_byte(const int32_t &val, const int &scale, const int &zero_offset)
{
  int raw_value = std::abs((val / scale) + zero_offset); // Get raw value, then clamp
  return std::min(std::max(raw_value, 0), 255);
}

void start_uros_node(){

  // Indicator led for attachment attempt
  for (int i = 0; i <= 7; i++) {
   digitalWrite(CONNECTED_TO_AGENT_PIN, !digitalRead(CONNECTED_TO_AGENT_PIN));
    delay(100);   
  }

  // Start the attachment attempt
  last_recv_msg_time = millis();
  
  set_microros_transports();
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  last_recv_msg_time = millis();
  RCCHECK(rclc_node_init_default(&node, "arduino_actuator_interface_node", "", &support));

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
            &cmd_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
            "arduino_cmd"));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
            &feedback_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
            "arduino_feedback"));

  // create timer
  last_recv_msg_time = millis();
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
            timer_callback));

  cmd_msg.data = 0;

  // create executor
  last_recv_msg_time = millis();
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

  last_recv_msg_time = millis();
}

void setup() {

  // Set default states for leds
  pinMode(ERROR_PIN, OUTPUT);
  digitalWrite(ERROR_PIN, HIGH);

  pinMode(CONNECTED_TO_AGENT_PIN, OUTPUT);
  digitalWrite(CONNECTED_TO_AGENT_PIN, LOW);

  // Make attachment attempt
  start_uros_node();
}


void loop() {

  // TODO: seems to enter error loop immediately when trying to re-attach after sitting for a long time
  // try using rmw_uros_ping_agent to check connection to agent, see https://micro.ros.org/docs/tutorials/programming_rcl_rclc/micro-ROS/

  delay(LOOP_PERIOD_MS);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(LOOP_PERIOD_MS));
}
