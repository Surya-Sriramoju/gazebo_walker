/**
 * @file walker.cpp
 * @author Sai Surya Sriramoju (saisurya@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <walker.h>

Walker::Walker(const std::string &node_name) : Node(node_name) {
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1,
      std::bind(&Walker::topic_callback, this, std::placeholders::_1));
}

void Walker::topic_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg) {
  if (obstacle_detected(msg)) {
    walkerbot_velocity_.linear.x = 0.0;
    walkerbot_velocity_.angular.z = 0.25;
  } else {
    walkerbot_velocity_.linear.x = 0.25;
    walkerbot_velocity_.angular.z = 0.0;
  }
  publisher_->publish(walkerbot_velocity_);
}

bool Walker::obstacle_detected(
    const sensor_msgs::msg::LaserScan::ConstPtr &msg) {
  // determining in extreme 25 degree of the view
  std::array<int, 51> angles = {
      0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,
      13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,
      335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347,
      348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359};

  // return bool if the object is 0.3m away from the robot
  for (auto i : angles) {
    if (msg->ranges.at(i) < 0.3) {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected");
      return true;
    }
  }
  RCLCPP_WARN(this->get_logger(), "Obstacle not detected");
  return false;
}

int main(int argc, char* argv[]) {
  // Initializing the rclcpp
  rclcpp::init(argc, argv);

  // Instantiating and spinning the node
  rclcpp::spin(std::make_shared<Walker>());

  // Shutdown
  rclcpp::shutdown();

  return 0;
}