#include "ros2.h"

rclc_executor_t executor_mpu6050_pub;
rcl_publisher_t publisher_mpu6050;
sensor_msgs__msg__Imu mpu6050_msg;



void mpu_publisher(){
  RCCHECK(rclc_publisher_init_default(&publisher_mpu6050,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), TOPIC_MPU6050));
  RCCHECK(rclc_executor_init(&executor_mpu6050_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_mpu6050_pub, &timer));
  Serial.println("Mpu publisher created");
}
