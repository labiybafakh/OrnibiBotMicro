#include <Arduino.h>
#include <iostream>
#include <micro_ros_platformio.h>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "INA219.hpp"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/u_int16_multi_array.h>  // Import the UInt8MultiArray message type

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__UInt16MultiArray msg;  // Update the message type

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

OrnibiBot robot;

// SBUS wing_left(&Serial1, true);
// SBUS wing_right(&Serial2, true);

IntervalTimer interpolation;
IntervalTimer sbus;
IntervalTimer sensor;

static uint16_t mid_left = 1500;
static uint16_t mid_right = 1500;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void interpolationHandler(){
  robot._flappingParam->amplitude = 10;
  robot._flappingParam->frequency = 3;
  robot._flappingParam->offset = 0;

  int8_t signal = robot.flappingPattern(sine);

  robot._wingPosition->desired_left = mid_left + signal;
  robot._wingPosition->desired_right = mid_right + signal;

  if(robot._flappingParam->time < robot.getFlapMs())  robot._flappingParam->time++;
  else  robot._flappingParam->time = 0;

}

void sbusHandler(){

    // wing_left.setPosition(robot._wingPosition->desired_left);
    // wing_right.setPosition(robot._wingPosition->desired_right);

}

void sensorHandler(){

}

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
        msg.data.data[1]= robot._wingPosition->desired_left;
        msg.data.data[2]= robot._wingPosition->desired_right;
        msg.data.data[3]= 0;
        msg.data.data[4]= 0;
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(460800);
  set_microros_serial_transports(Serial);
  delay(2000);

  interpolation.begin(interpolationHandler, 1000);
  sbus.begin(sbusHandler, 10000);
  sensor.begin(sensorHandler, 1000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
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
  msg.data.size = 5;
  msg.data.data = (uint16_t*)allocator.zero_allocate(5, sizeof(uint16_t), allocator.state);
  for (uint16_t i = 0; i < 5; i++) {
    msg.data.data[i] = i;
  }
}

void loop() {
  delay(1);
  rmw_uros_ping_agent(100,1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
