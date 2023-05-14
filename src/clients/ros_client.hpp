#ifndef SYNAPSE_ROS_CLIENT_HPP__
#define SYNAPSE_ROS_CLIENT_HPP__

#include <actuator_msgs/msg/detail/actuators__struct.hpp>
#include <memory>
#include <rclcpp/subscription_options.hpp>
#include <string>

#include "synapse_tinyframe/TinyFrame.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "actuator_msgs/msg/actuators.hpp"

#include "synapse_protobuf/actuators.pb.h"


class TcpClient;


class RosClient : public rclcpp::Node  {
  public:
    RosClient(const std::shared_ptr<TinyFrame> & tf);
    void tf_send(TF_Msg & frame) const;
    void publish_actuators(const Actuators & msg);
  private:
    std::shared_ptr<TinyFrame> tf_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_actuators_;
    void joy_callback(const sensor_msgs::msg::Joy & msg) const;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__
