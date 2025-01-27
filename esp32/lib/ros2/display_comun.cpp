#include "ros2.h"


rcl_subscription_t subscriber_display;

std_msgs__msg__Int8 msg_display;

void display_subiscriber(){
  RCCHECK(rclc_subscription_init_best_effort(&subscriber_display, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), TOPIC_DISPLAY));
  RCCHECK(rclc_executor_add_subscription(&executor_led_sub, &subscriber_display, &msg_display, &display_sub_callback, ON_NEW_DATA)); 
  Serial.println("Display subscriber created");
}


void display_sub_callback(const void *msgin) {

  const std_msgs__msg__Int8 *msg_led = (const std_msgs__msg__Int8 *)msgin;
  
  custom_draw(msg_led->data);
}


