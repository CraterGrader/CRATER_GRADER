#include <micro_ros_arduino.h>

#include <stdio.h>
#include <string>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float64_multi_array.h>


// NUM_HANDLES must be updated to reflect total number of subscribers + publishers
#define NUM_HANDLES 2

#define LED_PIN 13
#define LOOP_PERIOD_MS 10
// TIMER_TIMEOUT_MS appears to control the publishing period
#define TIMER_TIMEOUT_MS 20

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_subscription_t actuator_cmd_sub;
rcl_publisher_t debug_msg_pub;

std_msgs__msg__Float64MultiArray cmd_msg;
std_msgs__msg__String debug_msg;
bool cmd_msg_received = false;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void actuator_cmd_callback(const void * msgin)
{
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  cmd_msg = *msg;
  cmd_msg_received = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (cmd_msg_received) {
      String debug_str = "Currently received data: [";
      for (size_t i = 0; i < cmd_msg.data.size; ++i) {
        debug_str += String(cmd_msg.data.data[i]) + String(", ");
      }
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

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &actuator_cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
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

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, NUM_HANDLES, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &actuator_cmd_sub, &cmd_msg, &actuator_cmd_callback, ON_NEW_DATA));
}

void loop() {
  delay(LOOP_PERIOD_MS);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(LOOP_PERIOD_MS)));

}
