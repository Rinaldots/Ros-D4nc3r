#include "ros2.h"

#include "motor_controle.h"
rcl_subscription_t subscriber_motor;
rclc_executor_t executor_motor_sub;
geometry_msgs__msg__Twist msg_motor;

rclc_executor_t executor_odom_pub;
rcl_publisher_t odom_publisher;

int threshold = 190;

void motor_subiscriber(){
  RCCHECK(rclc_subscription_init_best_effort(&subscriber_motor,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),TOPIC_MOTOR));
  // create executor
  RCCHECK(rclc_executor_init(&executor_motor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_motor_sub, &subscriber_motor, &msg_motor, &MotorControll_callback, ON_NEW_DATA)); 
  
  Serial.println("Motor subscriber created");
  
}

void odom_publisher_setup(){
  RCCHECK(rclc_publisher_init_default( &odom_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),TOPIC_ODOM));
  Serial.println("Odometry publisher created");
}



void MotorControll_callback(const void* msg_in) {
  const geometry_msgs__msg__Twist *msg_motor = (const geometry_msgs__msg__Twist *)msg_in;
  float linearVelocity;
  float angularVelocity;
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity = msg_motor->linear.x;
  angularVelocity = msg_motor->angular.z;
  
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
  float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;
  
  //current wheel rpm is calculated
  float currentRpmL = leftWheel.getRpm();
  float currentRpmR = rightWheel.getRpm();
  
  Serial.print("CurrentRpmL: ");
  Serial.println(currentRpmL);
  Serial.print("CurrentRpmR: ");
  Serial.println(currentRpmR);

  //pid controlled is used for generating the pwm signal
  float actuating_signal_LW = leftWheel.pid(vL, currentRpmL);
  float actuating_signal_RW = rightWheel.pid(vR, currentRpmR);
  
  Serial.print("ActuatingSignal_LW: ");
  Serial.println(actuating_signal_LW);
  Serial.print("ActuatingSignal_RW: ");
  Serial.println(actuating_signal_RW);
  
  rightWheel.moveBase(actuating_signal_RW, threshold);
  leftWheel.moveBase(actuating_signal_LW, threshold);
  
}



