/**
 * @file walker.h
 * @author Sai Surya Sriramoju (saisurya@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <memory>

class Walker : public rclcpp::Node {
 public:
  Walker(const std::string &node_name = "walker");

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_;  // publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_;                          // subscriber
  geometry_msgs::msg::Twist walkerbot_velocity_;  // velocity of bot

  /**
   * @brief The callback funciton for subscribing to laser and publishing
   * velocity.
   *
   * @param msg The laser scan from the /scan topic.
   */
  void topic_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg);

  /**
   * @brief Method to detect the obstacles.
   *
   * @param msg The msg from the /scan topic.
   * @return true if obstacle is detected.
   */
  bool obstacle_detected(const sensor_msgs::msg::LaserScan::ConstPtr &msg);
};
