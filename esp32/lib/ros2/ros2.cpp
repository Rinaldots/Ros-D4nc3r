#include "ros2.h"

// create a global shift register object
// parameters: <number of shift registers> (data pin, clock pin, latch pin)
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;

Odometry odometry;
nav_msgs__msg__Odometry odom_msg;

unsigned long prev_odom_update = 0;
float wheels_y_distance_ = 0.17;
float wheel_radius = 0.0335;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;
unsigned long long time_offset = 0;

void syncTime() {
    // get the current time from the agent
    rmw_uros_sync_session(1000);
    unsigned long now = millis();
     // Ensure this function is available
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime() {
    struct timespec tp = { 0 };
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void ros_setup() {    
    // Adding Wifi192.168.1.39
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
    
    //timer
    led_subiscriber();
    display_subiscriber();
    odom_publisher_setup();
    motor_subiscriber();
    mpu_publisher();
    
    timer_setup();

    syncTime();
    
}


void timer_setup(){
  const unsigned int timer_encoder_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(timer_encoder_timeout),timer_callback));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer != NULL){
    MPU6050Values mpu_value = get_mpu6050_value(); 
    struct timespec time_stamp = getTime();
    unsigned long now = millis();
    mpu6050_msg.header.frame_id = micro_ros_string_utilities_set(mpu6050_msg.header.frame_id, "odom");
    mpu6050_msg.header.stamp.sec = time_stamp.tv_sec;
    mpu6050_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    mpu6050_msg.linear_acceleration.x = mpu_value.gyro_x;
    mpu6050_msg.linear_acceleration.y = mpu_value.gyro_y;
    mpu6050_msg.linear_acceleration.z = mpu_value.gyro_z;
    mpu6050_msg.angular_velocity.x = mpu_value.accel_x;
    mpu6050_msg.angular_velocity.y = mpu_value.accel_y;
    mpu6050_msg.angular_velocity.z = mpu_value.accel_z;
    RCSOFTCHECK(rcl_publish(&publisher_mpu6050, &mpu6050_msg, NULL));
    //odometry
    float currentRpmL = leftWheel.getRpm_real();
    
    float currentRpmR = rightWheel.getRpm_real();
    float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 20.0;  // RPM
    float linear_x = average_rps_x * wheel_circumference_;                  // m/s
    float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 20.0;
    float angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0);  //  rad/s
    float linear_y = 0;
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
    vel_dt,
    linear_x,
    linear_y,
    angular_z);
    odom_msg = odometry.getData();
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));


  }
}

void ros2_loop() {
  
  RCCHECK(rclc_executor_spin_some(&executor_led_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_motor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_mpu6050_pub, RCL_MS_TO_NS(100)));
  
}

