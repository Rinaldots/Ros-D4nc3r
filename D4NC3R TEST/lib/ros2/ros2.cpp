#include "ros2.h"


rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;

unsigned long prev_odom_update = 0;
float wheels_y_distance_ = 0.17;
float wheel_radius = 0.0335;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;
unsigned long long time_offset = 0;

void syncTime() {
    rmw_uros_sync_session(1000);
    unsigned long now = millis();
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime() {
    struct timespec tp = { 0 };
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void ros_setup() {    

    IPAddress agent_ip(192, 168, 1, 123);
    size_t agent_port = 8888;
    char ssid[] = "10D";
    char psk[]= "12345678";
    
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to WiFi");

    allocator = rcl_get_default_allocator();
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "baile", NAMESPACE, &support));

    
    motor_subiscriber();

    timer_setup();

    encoder_publisher_setup();

    syncTime();

    //rclc_executor_spin(&executor_encoder_pub);
}

void timer_setup(){
  const unsigned int timer_encoder_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(timer_encoder_timeout),timer_callback));
}


void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer != NULL){
        std_msgs__msg__Int32 left_encoder_msg;
        std_msgs__msg__Int32 right_encoder_msg;
        left_encoder_msg.data = leftWheel.Enc_count;
        right_encoder_msg.data = rightWheel.Enc_count;
        RCSOFTCHECK(rcl_publish(&publisher_left_encoder, &left_encoder_msg, NULL));
        RCSOFTCHECK(rcl_publish(&publisher_right_encoder, &right_encoder_msg, NULL));
        //Serial.print(right_encoder_msg.data);
        
    }
}


void ros2_loop() {
  RCCHECK(rclc_executor_spin_some(&executor_motor_sub, RCL_MS_TO_NS(100)));
  leftWheel.Calcular_Velocidade();
  rightWheel.Calcular_Velocidade();

  // Calculate PID for both motors
  calculate_pid_left_motor();
  calculate_pid_right_motor();

  // Apply PWM values
  leftWheel.pwm(leftWheel.PWM_value, 1);
  rightWheel.pwm(rightWheel.PWM_value, 1);

  // Debugging output
  Serial.print("Left Wheel Velocity: ");
  Serial.println(leftWheel.Velocidade_dv);
  Serial.print("Right Wheel Velocity: ");
  Serial.println(rightWheel.Velocidade_dv);
}

