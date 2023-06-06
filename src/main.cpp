#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int8_multi_array.h>  // Import the UInt8MultiArray message type

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__UInt8MultiArray msg;  // Update the message type

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

IntervalTimer interpolation;
IntervalTimer sbus;
IntervalTimer sensor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    
    // Update each element in the array
        msg.data.data[0]= millis();
    for (uint8_t i = 1; i < msg.data.size; i++) {
      msg.data.data[i]++;
    }
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(460800);
  set_microros_serial_transports(Serial);
  delay(2000);
//   interpolation.begin

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "micro_ros_platformio_node_publisher"));

  const unsigned int timer_timeout = 1;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize the UInt8MultiArray
  msg.data.size = 16;
  msg.data.data = (uint8_t*)allocator.zero_allocate(16, sizeof(uint8_t), allocator.state);
  for (uint8_t i = 0; i < 16; i++) {
    msg.data.data[i] = i;
  }
}

void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
