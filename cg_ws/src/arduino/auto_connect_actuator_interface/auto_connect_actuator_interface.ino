// Modified from https: // github.com/micro-ROS/micro_ros_arduino/blob/1df47435f08b9609effaec9cb0cc99241ff9dc30/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <cmath> // std::abs
#include <algorithm> // std::min and std::max
#include <std_msgs/msg/int64.h>
#include "RoboClaw.h"


/* MicroROS declarations */
// NUM_HANDLES must be updated to reflect total number of subscribers + publishers
#define NUM_HANDLES 2

#define LED_PIN 13
#define CONN_PIN 12
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }

// Declare microros objects
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t feedback_pub;
rcl_subscription_t cmd_sub;
std_msgs__msg__Int64 feedback_msg;
std_msgs__msg__Int64 cmd_msg;
bool micro_ros_init_successful;

/* Roboclaw declarations */
// Roboclaw constants
#define BYTE_TO_QPPS_DRIVE_SCALE 25  // Drive Scale 
#define BYTE_TO_QP_STEER_SCALE 22 // Steer Scale
#define BYTE_TO_QPPS_DRIVE_STEER_OFFSET 127 // 
#define BYTE_TO_QP_DELTA_POS_SCALE 10  // Drive Scale 
#define BYTE_TO_QP_DELTA_POS_OFFSET 127 // 

#define BYTE_TO_QP_TOOL_SCALE 129 // Tool Scale = floor( (Full Scale QP as flashed on Roboclaw) / 255 )
#define BYTE_TO_QP_TOOL_OFFSET 0 // Tool offset 

#define POSN_CTRL_ACCEL_QPPS 1000
#define POSN_CTRL_DECCEL_QPPS 1000
#define POSN_CTRL_SPD_QPPS 1300

#define DRIVE_CTRL_ACCEL_QPPS 5000

#define TOOL_CTRL_ACCEL_QPPS 7000
#define TOOL_CTRL_DECCEL_QPPS 5000
#define TOOL_CTRL_SPD_QPPS 10000

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

// delta position
int32_t deltaPosM1Last = 0;
int32_t deltaPosM1Curr = 0;
int32_t deltaPosM2Last = 0;
int32_t deltaPosM2Curr = 0;

// subscription callback
bool cmd_msg_received = false;

// interface setup
#define ROBOCLAW_ADDRESS 0x80
#define NUM_ROBOCLAWS_MOBILITY 2
#define NUM_ROBOCLAWS_TOOL 1
#define ROBOCLAW_READ_TIMEOUT_USEC 10000 // 8k --> ~3.3hz, lower = faster loop but below 8k observed to not successfully read from the RoboClaws (tested 7k, 6k, 5k, 100)
RoboClaw roboclaws_mobility[] = {
  RoboClaw(&Serial1, ROBOCLAW_READ_TIMEOUT_USEC), // Pins 18(Tx) and 19(Rx) on the Due
  RoboClaw(&Serial2, ROBOCLAW_READ_TIMEOUT_USEC) // Pins 16(Tx) and 17(Rx) on the Due
};
RoboClaw roboclaws_tool[] = {
  RoboClaw(&Serial3, ROBOCLAW_READ_TIMEOUT_USEC) // Pins 14(Tx) and 15(Rx) on the Due
};
// Rear RoboClaw has sign flipped
int roboclaw_signs[] = {
  -1, 1
};


/* Callbacks */
void cmd_callback(const void * msgin)
{
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
  cmd_msg = *msg;
  cmd_msg_received = true;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
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
        roboclaws_mobility[i].SpeedAccelM1(ROBOCLAW_ADDRESS, DRIVE_CTRL_ACCEL_QPPS, roboclaw_signs[i]*drive_cmd); // speed command uses QPPS ramping
        roboclaws_mobility[i].SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS, POSN_CTRL_ACCEL_QPPS, POSN_CTRL_SPD_QPPS, POSN_CTRL_DECCEL_QPPS, roboclaw_signs[i]*steer_cmd, 1);
      }

      // Send Tool Commands
      for (int i = 0; i < NUM_ROBOCLAWS_TOOL; ++i) {
        roboclaws_tool[i].SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, TOOL_CTRL_ACCEL_QPPS, TOOL_CTRL_SPD_QPPS, TOOL_CTRL_DECCEL_QPPS, tool_cmd, 1);
      }

    } // if (cmd_msg_received)

    /* Read Encoder Values */
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
    uint8_t drive_delta_pos_front = int32_to_byte(deltaPosM1Curr-deltaPosM1Last, BYTE_TO_QP_DELTA_POS_SCALE, BYTE_TO_QP_DELTA_POS_OFFSET);
    deltaPosM1Last = deltaPosM1Curr;

    // Drive delta position rear
    deltaPosM2Curr = roboclaws_mobility[1].ReadEncM1(ROBOCLAW_ADDRESS, &rc_1_drive_status, &rc_1_drive_valid);
    uint8_t drive_delta_pos_rear = int32_to_byte(deltaPosM2Curr - deltaPosM2Last, BYTE_TO_QP_DELTA_POS_SCALE, BYTE_TO_QP_DELTA_POS_OFFSET);
    deltaPosM2Last = deltaPosM2Curr;

    // Terminal byte (limit switches, heartbeat, etc.)
    uint8_t term_byte = 0;

    // [Steer 1 Encoder, Steer 2 Encoder, Tool Encoder Value, Drive 1 Speed, Drive 2 Speed, Drive Delta Pos Front, Drive Delta Pos Rear, Term Byte]
    feedback_msg.data = (static_cast<uint64_t>(term_byte) << 56) | (static_cast<uint64_t>(drive_delta_pos_rear) << 48) |(static_cast<uint64_t>(drive_delta_pos_front) << 40) | (static_cast<uint64_t>(R2spd1Scale) << 32) | (static_cast<uint64_t>(R1spd1Scale) << 24) | (static_cast<uint64_t>(R3enc1Scale) << 16) | (static_cast<uint64_t>(R2enc2Scale) << 8) | static_cast<uint64_t>(R1enc2Scale);
    
    rcl_publish(&feedback_pub, &feedback_msg, NULL);
  } // if (timer != NULL)
} // timer_callback()

/* Helper functions */
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

/* MicroROS functions */
// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "arduino_actuator_interface_node", "", &support));

  // create publisher
//  rclc_publisher_init_best_effort
//rclc_publisher_init_default
  RCCHECK(rclc_publisher_init_default(
      &feedback_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
      "arduino_feedback"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
      "arduino_cmd"));

  // create timer
  const unsigned int timer_period_ms = 10;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_period_ms),
      timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, NUM_HANDLES, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA));

  micro_ros_init_successful = true;
}

void destroy_entities()
{
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_publisher_fini(&feedback_pub, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

void setup()
{
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(CONN_PIN, OUTPUT);
  digitalWrite(CONN_PIN, LOW);

  micro_ros_init_successful = false;
  feedback_msg.data = 0;

  // Set up RoboClaws
  for (auto & roboclaw : roboclaws_mobility) {
    roboclaw.begin(38400);
  }
  for (auto & roboclaw : roboclaws_tool) {
    roboclaw.begin(38400);
  }
}

uint32_t delay_ms = 500; // short delay to blink the red LED while trying to connect
void loop()
{
  // Keep trying to connect by pinging the MicroROS agent
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2))
  {
    // Use flag to see if entities need to be created
    if (!micro_ros_init_successful)
    {
      create_entities();
    }
    else
    {
      // Main loop to run the MicroROS node
      digitalWrite(CONN_PIN, HIGH); // Green LED on
      digitalWrite(LED_PIN, LOW); // Red LED off
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }
  }
  else {
    // Destroy entities if there is not connection to the agent
    if (micro_ros_init_successful)
    {
      destroy_entities();
      digitalWrite(CONN_PIN, LOW); // Green LED off
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink red LED while trying to connect
    delay(delay_ms);
  }
}
