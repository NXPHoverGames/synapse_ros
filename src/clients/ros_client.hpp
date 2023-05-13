#ifndef SYNAPSE_ROS_CLIENT_HPP__
#define SYNAPSE_ROS_CLIENT_HPP__

#include <memory>
#include <rclcpp/subscription_options.hpp>
#include <string>

#include "synapse_tinyframe/TinyFrame.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"


class TcpClient;


class RosClient : public rclcpp::Node  {
  public:
    RosClient(const std::shared_ptr<TinyFrame> & tf);
    void tf_send(TF_Msg & frame) const;
  private:
    std::shared_ptr<TinyFrame> tf_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    void joy_callback(const sensor_msgs::msg::Joy & msg) const;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__
