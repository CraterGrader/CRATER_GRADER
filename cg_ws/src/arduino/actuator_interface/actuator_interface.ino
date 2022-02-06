#include <micro_ros_arduino.h>

#include <stdio.h>
#include <string>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>

#include "RoboClaw.h"

// NUM_HANDLES must be updated to reflect total number of subscribers + publishers
#define NUM_HANDLES 2

#define LED_PIN 13
#define LOOP_PERIOD_MS 10
// TIMER_TIMEOUT_MS appears to control the publishing period
#define TIMER_TIMEOUT_MS 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_subscription_t cmd_wheel_vel_sub;
rcl_publisher_t debug_msg_pub;

std_msgs__msg__Float32 cmd_wheel_vel_msg;
std_msgs__msg__String debug_msg;
bool cmd_msg_received = false;

/* RoboClaws */
#define ROBOCLAW_ADDRESS 0x80
#define NUM_ROBOCLAWS 1
RoboClaw roboclaws[] = {
  RoboClaw(&Serial1,10000) // Pins 18 and 19 on the Due
};

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void cmd_wheel_vel_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  cmd_wheel_vel_msg = *msg;
  cmd_msg_received = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (cmd_msg_received) {
      // Command RoboClaws
      int wheel_pwm = (cmd_wheel_vel_msg.data + 100.0) / (200.0) * (127); // Map [-100, 100] to [0, 127] // TODO define these as constants
      if (abs(wheel_pwm - 64) <= 1) wheel_pwm = 64;  // Fix to zero when close to zero
      for (int i = 0; i < NUM_ROBOCLAWS; ++i) {
        roboclaws[i].ForwardBackwardM1(ROBOCLAW_ADDRESS, wheel_pwm);
        roboclaws[i].ForwardBackwardM2(ROBOCLAW_ADDRESS, wheel_pwm);
      }

      String debug_str = "Currently received data: " + String(cmd_wheel_vel_msg.data);
      debug_msg.data.data = const_cast<char*>(debug_str.c_str());
    } else {
      debug_msg.data.data = const_cast<char*>("No data received");
    }
    RCSOFTCHECK(rcl_publish(&debug_msg_pub, &debug_msg, NULL));
  }
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

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
    &cmd_wheel_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "arduino_cmd_wheel_vel"));

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

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, NUM_HANDLES, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_wheel_vel_sub, &cmd_wheel_vel_msg, &cmd_wheel_vel_callback, ON_NEW_DATA));

  // Set up RoboClaws
  for (auto & roboclaw : roboclaws) {
    roboclaw.begin(38400);
  }

}

void loop() {
  delay(LOOP_PERIOD_MS);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(LOOP_PERIOD_MS)));

}
