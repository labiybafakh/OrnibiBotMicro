#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "Thread.h"
#include "ThreadController.h"


SBUS wing_left(&Serial1, true);
SBUS wing_right(&Serial2, true);
SBUS tail_pitch(&Serial4, true);
SBUS tail_roll(&Serial5, true);

OrnibiBot robot;

int targetServo[5];

void setup()
{
  Serial.begin(115200);
  
}

void loop()
{
  robot._flapFreq = 5;
  robot._amplitude = 30;
  robot._offset = 0;

  targetServo[0] = robot.flappingPattern(sine);
  targetServo[1] = robot.flappingPattern(sine);
    Serial.println(targetServo[0]);

  wing_left.setPosition(targetServo);
  wing_left.sendPosition();

  wing_right.setPosition(targetServo);
  wing_right.sendPosition();

  if (robot._time < robot._periode)
    robot._time++;
  else
    robot._time = 0;

  delay(10);
}

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>

// rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCCHECK(fn)                                                                                                    \
//   {                                                                                                                    \
//     rcl_ret_t temp_rc = fn;                                                                                            \
//     if ((temp_rc != RCL_RET_OK))                                                                                       \
//     {                                                                                                                  \
//       error_loop();                                                                                                    \
//     }                                                                                                                  \
//   }
// #define RCSOFTCHECK(fn)                                                                                                \
//   {                                                                                                                    \
//     rcl_ret_t temp_rc = fn;                                                                                            \
//     if ((temp_rc != RCL_RET_OK))                                                                                       \
//     {                                                                                                                  \
//     }                                                                                                                  \
//   }

// void get_teensy_mac(uint8_t *mac) {
//     for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
//     for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
// }

// // Error handle loop
// void error_loop()
// {
//   while (1)
//   {
//     delay(1);
//   }
// }

// void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
// {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL)
//   {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data = analogRead(A0);
//   }
// }

// void setup()
// {
//   // Configure serial transport
//   Serial.begin(115200);
//   pinMode(A0, INPUT);
//   analogReadResolution(12);

//   byte local_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
//   get_teensy_mac(local_mac);
//   IPAddress local_ip(192, 168, 30, 210);
//   IPAddress agent_ip(192, 168, 30, 243);
//   size_t agent_port = 8888;

//   set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
//   delay(2000);

//   allocator = rcl_get_default_allocator();

//   // create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

//   // create publisher
//   RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//                                       "micro_ros_platformio_node_publisher"));

//   // create timer,
//   const unsigned int timer_timeout = 2;
//   RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));

//   msg.data = 0;
// }

// void loop()
// {
//   delay(1);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
// }