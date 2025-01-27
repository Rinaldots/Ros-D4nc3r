#include "ros2.h"

rcl_subscription_t subscriber_led;
rclc_executor_t executor_led_sub;
std_msgs__msg__Int32 msg_led;


ShiftRegister74HC595<1> sr(14, 13, 12);

void led_subiscriber(){
  RCCHECK(rclc_subscription_init_best_effort(&subscriber_led,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),TOPIC_LED));
  // create executor
  RCCHECK(rclc_executor_init(&executor_led_sub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_led_sub, &subscriber_led, &msg_led, &subscription_callback, ON_NEW_DATA)); 
  Serial.println("Led subscriber created");
}

std::string padString(const std::string &input) {
    int totalLength = 8;
    int inputLength = input.length();
    int zerosToAdd = totalLength - inputLength;

    std::string paddedString = std::string(zerosToAdd, '0') + input;
    return paddedString;
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg_led = (const std_msgs__msg__Int32 *)msgin;
  // (condition) ? (true exec):(false exec)
  int32_t led_state = msg_led->data;
  
  std::string str = std::to_string(led_state);
  std::string result = padString(str);

  for(int i = 1; i < 9; i++){
      if(result[(i-1)] == '0' ){
        sr.set((i), LOW);
      }else{
        sr.set((i), HIGH);
  }
  }
  
  
}