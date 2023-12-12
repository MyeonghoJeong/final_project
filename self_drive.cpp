#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
  int step_;
  
public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", lidar_qos_profile,
                                                                       callback);
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::Twist vel;
    double front_distance = scan->ranges[0];
    double diagonal_left_distance1 = scan->ranges[45];
    double diagonal_left_distance2 = scan->ranges[135];
    double left_vertical= scan->ranges[90];
    
       
    if (front_distance <= 0.35)
    {
      vel.linear.x = 0.0;
      vel.angular.z = -1.0;
    }
    else
    {
      vel.linear.x = 0.15;

      if (left_vertical <= 0.2)
      {
        vel.linear.x = 0.15;
        vel.angular.z = 0.0;
      }
      else if (diagonal_left_distance1 > diagonal_left_distance2)
      {
        vel.linear.x = 0.1;
        vel.angular.z = 1.0;
      }
      else if (diagonal_left_distance2 > diagonal_left_distance1)
      {
        vel.linear.x = 0.1;
        vel.angular.z = -1.0;
      }
      
      
      
    }
  
   std::cout <<"test09" <<std::endl;
    RCLCPP_INFO(rclcpp::get_logger("self_drive"),
                "step=%d, range=%1.2f, linear=%1.2f, angular=%1.2f", step_, scan->ranges[0],
                vel.linear.x, vel.angular.z);
    pose_pub_->publish(vel);
    step_++;
  }
  
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
