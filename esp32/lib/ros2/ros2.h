#ifndef LED_COMUN_H
#define LED_COMUN_H


#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <string>
#include <WiFi.h>
#include <ShiftRegister74HC595.h>

#define LED_PIN 2
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn;if (temp_rc != RCL_RET_OK) { while (1){digitalWrite(LED_PIN,!digitalRead(LED_PIN));delay(100);}}}
#define RCSOFTCHECK(fn){rcl_ret_t temp_rc = fn;if ((temp_rc != RCL_RET_OK)){    Serial.println("Error in function");    }}
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define CONCAT(a, b, c) a b c

#include "display.h"
#include "encoder.h"
#include "mpu6050.h"
#include "odometry.h"
#include "motor_controle.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
// Declare the shared variables as extern

// Topicos
#define D4NC3RNUMBER 1
#define NAMESPACE CONCAT("/d4nc3r", TOSTRING(D4NC3RNUMBER), "")
#define TOPIC_LED CONCAT("micro_ros", "", "/led")
#define TOPIC_DISPLAY CONCAT("micro_ros", "","/display")
#define TOPIC_MPU6050 CONCAT("micro_ros", "","/mpu6050")
#define TOPIC_MOTOR CONCAT("micro_ros", "","/cmd_vel")
#define TOPIC_ODOM CONCAT("micro_ros", "","/odom")

extern rcl_node_t node;
extern rclc_support_t support;
extern rcl_allocator_t allocator;

//Subiscriber declaration
extern rcl_subscription_t subscriber_led;
extern rclc_executor_t executor_led_sub;

extern rcl_subscription_t subscriber_display;
extern rclc_executor_t executor_display_sub;

extern rcl_subscription_t subscriber_motor;
extern rclc_executor_t executor_motor_sub;


//Publisher declaration

extern rclc_executor_t executor_odom_pub;
extern rcl_publisher_t odom_publisher;

extern rclc_executor_t executor_mpu6050_pub;
extern rcl_publisher_t publisher_mpu6050;


//Timer declaration
extern rcl_timer_t timer;

//Message declaration
extern std_msgs__msg__Int32 msg_led;
extern std_msgs__msg__Int8 msg_display;
extern sensor_msgs__msg__Imu mpu6050_msg;

// Declare the shared functions
void ros_setup();
void ros2_loop();

void timer_setup();
void timer_callback(rcl_timer_t *timer, int64_t last_call_time); 

void led_subiscriber();
std::string padString(const std::string &input);
void subscription_callback(const void *msgin);

void motor_subiscriber();
void subscription_motor_callback(const void *msgin);

void display_subiscriber();
void display_sub_callback(const void *msgin);

void mpu_publisher();

void display_subiscriber();

void odom_publisher_setup();
void publishData();

void MotorControll_callback(const void *msgin);

void encoder_setup();

// Funções de sincronização de tempo
void syncTime();
struct timespec getTime();

extern unsigned long long time_offset;
#endif // LED_COMUN_H