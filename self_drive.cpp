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
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
    int step_;
    double safety_distance_;
    double obstacle_passing_distance_;
    double wall_following_distance_;

    bool isWallDetected(double min_distance)
    {
        return min_distance < safety_distance_;
    }

    bool isObstacleBetweenWalls(double min_distance_left, double min_distance_right)
    {
        return min_distance_left > obstacle_passing_distance_ && min_distance_right > obstacle_passing_distance_;
    }

    bool isCloseToWall(double min_distance)
    {
        return min_distance < wall_following_distance_;
    }

    bool isWallInSight(double min_distance_left, double min_distance_left_diagonal, double min_distance_front, double min_distance_right_diagonal, double min_distance_right)
    {
        return min_distance_left < safety_distance_ || min_distance_left_diagonal < safety_distance_ ||
               min_distance_front < safety_distance_ || min_distance_right_diagonal < safety_distance_ ||
               min_distance_right < safety_distance_;
    }

    void moveAlongWall(geometry_msgs::msg::Twist &vel)
    {
        vel.linear.x = 0.15;
        vel.angular.z = 0.0;
    }

    void stopAndRotate(geometry_msgs::msg::Twist &vel)
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.2;
    }

    void stopAndReverse(geometry_msgs::msg::Twist &vel)
    {
        vel.linear.x = -0.1;
        vel.angular.z = 0.0;
    }

    void handleWallDetection(const sensor_msgs::msg::LaserScan::SharedPtr scan, geometry_msgs::msg::Twist &vel)
    {
        double min_distance = *std::min_element(scan->ranges.begin(), scan->ranges.end());

        if (isWallDetected(min_distance))
        {
            handleWallFollow(scan, vel);
        }
        else
        {
            stopAndRotate(vel);
        }
    }

    void handleWallFollow(const sensor_msgs::msg::LaserScan::SharedPtr scan, geometry_msgs::msg::Twist &vel)
    {
        double min_distance = *std::min_element(scan->ranges.begin(), scan->ranges.end());

        double angle_min = scan->angle_min;
        double angle_increment = scan->angle_increment;

        double left_angle = -45.0;
        double left_diagonal_angle = -22.5;
        double front_angle = 0.0;
        double right_diagonal_angle = 22.5;
        double right_angle = 45.0;

        int left_index = static_cast<int>((left_angle - angle_min) / angle_increment);
        int left_diagonal_index = static_cast<int>((left_diagonal_angle - angle_min) / angle_increment);
        int front_index = static_cast<int>((front_angle - angle_min) / angle_increment);
        int right_diagonal_index = static_cast<int>((right_diagonal_angle - angle_min) / angle_increment);
        int right_index = static_cast<int>((right_angle - angle_min) / angle_increment);

        double min_distance_left = *std::min_element(scan->ranges.begin() + left_index, scan->ranges.begin() + left_index + 1 + 2 * static_cast<int>((5.0 / angle_increment)));
        double min_distance_left_diagonal = *std::min_element(scan->ranges.begin() + left_diagonal_index, scan->ranges.begin() + left_diagonal_index + 1 + 2 * static_cast<int>((5.0 / angle_increment)));
        double min_distance_front = *std::min_element(scan->ranges.begin() + front_index, scan->ranges.begin() + front_index + 1 + 2 * static_cast<int>((5.0 / angle_increment)));
        double min_distance_right_diagonal = *std::min_element(scan->ranges.begin() + right_diagonal_index, scan->ranges.begin() + right_diagonal_index + 1 + 2 * static_cast<int>((5.0 / angle_increment)));
        double min_distance_right = *std::min_element(scan->ranges.begin() + right_index, scan->ranges.begin() + right_index + 1 + 2 * static_cast<int>((5.0 / angle_increment)));

        if (isObstacleBetweenWalls(min_distance_left, min_distance_right))
        {
            moveAlongWall(vel);
        }
        else if (isCloseToWall(min_distance))
        {
            moveAlongWall(vel);
        }
        else
        {
            moveAlongWall(vel);
        }
    }

public:
    SelfDrive() : rclcpp::Node("self_drive"), step_(0), safety_distance_(0.25), obstacle_passing_distance_(0.35), wall_following_distance_(0.35)
    {
        auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
        lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", lidar_qos_profile, callback);

        auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);
    }

    void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        geometry_msgs::msg::Twist vel;

        handleWallDetection(scan, vel);

        // 로봇의 동작 결정 ...

        RCLCPP_INFO(rclcpp::get_logger("self_drive"), "step=%d, linear=%1.2f, angular=%1.2f", step_, vel.linear.x, vel.angular.z);
        pose_pub_->publish(vel);
        step_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SelfDrive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
