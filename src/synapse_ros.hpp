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


void tcp_entry_point();


class SynapseRos : public rclcpp::Node  {
  public:
    SynapseRos();
    virtual ~SynapseRos();
    void tf_send(TF_Msg & frame) const;
    void publish_actuators(const Actuators & msg);
  private:
    std::shared_ptr<TinyFrame> tf_{TF_Init(TF_MASTER)};
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_actuators_;
    void joy_callback(const sensor_msgs::msg::Joy & msg) const;
    std::shared_ptr<std::thread> tcp_thread_;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__
