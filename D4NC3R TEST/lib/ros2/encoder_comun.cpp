#include "ros2.h"

rclc_executor_t executor_encoder_pub;
rcl_publisher_t publisher_left_encoder;
rcl_publisher_t publisher_right_encoder;


void encoder_publisher_setup(){
  RCCHECK(rclc_publisher_init_default( &publisher_left_encoder,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),TOPIC_ENC_LEFT));
  RCCHECK(rclc_publisher_init_default( &publisher_right_encoder,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),TOPIC_ENC_RIGHT));
  RCCHECK(rclc_executor_init(&executor_encoder_pub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_encoder_pub, &timer));
  
  Serial.println("Encoder publisher created");
}