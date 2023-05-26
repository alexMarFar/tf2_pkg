/* Copyright (C) - All Rights Reserved

  Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
  Licensed under the Apache License, Version 2.0
*/
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>

class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode() : Node("tf2_listener_node")
  {
    // Get the topic name parameter value
    target_frame_ = declare_parameter<std::string>("target_frame", "target_frame");
    source_frame_ = declare_parameter<std::string>("source_frame", "source_frame");
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf2_transforms", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ListenerNode::publishTransforms, this));
  }

private:
  void publishTransforms()
  {
    tf2_msgs::msg::TFMessage msg;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    try
    {
      // Get transforms from tf buffer
      geometry_msgs::msg::TransformStamped transform;
      try
      {
        transform = tf_buffer_->lookupTransform("target_frame", "source_frame", tf2::TimePointZero);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        return;
      }

      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      transforms.push_back(transform);

    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
      return;
    }

    // Add transforms to the message
    for (const auto &transform : transforms)
    {
      msg.transforms.push_back(transform);
    }

    // Publish the transforms
    tf_publisher_->publish(msg);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string target_frame_, source_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}

