#include "ros2.h"

#include "motor_controle.h"
rcl_subscription_t subscriber_motor;
rclc_executor_t executor_motor_sub;
geometry_msgs__msg__Twist msg_motor;

int threshold = 190;

void motor_subiscriber(){
  RCCHECK(rclc_subscription_init_best_effort(&subscriber_motor,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),TOPIC_MOTOR));
  // create executor
  RCCHECK(rclc_executor_init(&executor_motor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_motor_sub, &subscriber_motor, &msg_motor, &MotorControll_callback, ON_NEW_DATA)); 

  Serial.println("Motor subscriber created");
  
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
  
  
  leftWheel.RPM_target = vL;
  rightWheel.RPM_target = vR;
 
  leftWheel.pwm(leftWheel.PWM_value, 1);
  rightWheel.pwm(rightWheel.PWM_value, 1);
  
  Serial.print("Left Wheel PWM: ");
  Serial.println(rightWheel.PWM_value);
  Serial.print("Left Wheel RPM target: ");
  Serial.println(rightWheel.RPM_target);
  Serial.print("Left Wheel RPM: ");
  Serial.println(rightWheel.Velocidade_dv);
  
}



