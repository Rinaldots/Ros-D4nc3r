#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"  // Adicionar este include
#include "nav_msgs/msg/odometry.hpp" // Alterado de geometry_msgs::msg::Pose
#include "visualization_msgs/msg/marker_array.hpp" // Alterado de Marker para MarkerArray

using namespace std::chrono_literals;
class YoloToTwist : public rclcpp::Node
{
  public:
    YoloToTwist()
    : Node("yolo_to_twist")
    {
      this->declare_parameter<std::string>("child_frame_id", "base_link");
      this->declare_parameter<std::string>("header_frame_id", "odom");
      this->get_parameter("child_frame_id", child_frame_id_);
      this->get_parameter("header_frame_id", header_frame_id_);

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("yolo_cords", 30); // Alterado para nav_msgs::msg::Odometry
      subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>( // Alterado para MarkerArray
        "marker", 30,
        
        std::bind(&YoloToTwist::marker_callback, this, std::placeholders::_1));
    }

  private:
    void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) // Alterado para MarkerArray
    {
      for (const auto& marker : msg->markers)
      {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now(); 
        odom_msg.header.frame_id = header_frame_id_; 
        odom_msg.child_frame_id = child_frame_id_; 

        odom_msg.pose.pose = marker.pose; 
        odom_msg.pose.covariance = {
          0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  // Variância em x (posição)
          0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  // Variância em y (posição)
          0.0,  0.0,  0.001,0.0,  0.0,  0.0,  // Variância em z (posição)
          0.0,  0.0,  0.0,  0.0, 0.0,  0.0,  // Variância em rotação x (roll)
          0.0,  0.0,  0.0,  0.0,  0.0, 0.0,  // Variância em rotação y (pitch)
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0  // Variância em rotação z (yaw)
        };

        // Se você também tivesse estimativas de velocidade, preencheria o twist e sua covariância:
        // odom_msg.twist.twist = ...;
        // odom_msg.twist.covariance = {
        //   vx_var, 0, ...
        //   ...
        // };
        publisher_->publish(odom_msg); 
      }
    }
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_; // Alterado para nav_msgs::msg::Odometry
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscriber_; // Alterado para MarkerArray
    std::string child_frame_id_;
    std::string header_frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloToTwist>());
  rclcpp::shutdown();
  return 0;
}