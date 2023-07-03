#ifndef SYNAPSE_ROS_CLIENT_HPP__
#define SYNAPSE_ROS_CLIENT_HPP__

#include "rclcpp/rclcpp.hpp"

#include "synapse_tinyframe/TinyFrame.h"
#include <memory>
#include <rclcpp/subscription_options.hpp>

#include "actuator_msgs/msg/actuators.hpp"
#include "synapse_protobuf/actuators.pb.h"

#include "nav_msgs/msg/odometry.hpp"
#include "synapse_protobuf/odometry.pb.h"

#include "sensor_msgs/msg/joy.hpp"
#include "synapse_protobuf/joy.pb.h"

#include "synapse_msgs/msg/bezier_trajectory.hpp"
#include "synapse_protobuf/bezier_trajectory.pb.h"

#include "geometry_msgs/msg/twist.hpp"
#include "synapse_protobuf/twist.pb.h"

class TcpClient;

void tcp_entry_point();

class SynapseRos : public rclcpp::Node {
public:
    SynapseRos();
    virtual ~SynapseRos();
    void tf_send(int topic, const std::string& data) const;
    void publish_actuators(const synapse::msgs::Actuators& msg);
    void publish_odometry(const synapse::msgs::Odometry& msg);

private:
    std::shared_ptr<TinyFrame> tf_ { TF_Init(TF_MASTER) };

    // subscriptions ros -> cerebri
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr sub_actuators_;
    void actuators_callback(const actuator_msgs::msg::Actuators& msg) const;

    rclcpp::Subscription<synapse_msgs::msg::BezierTrajectory>::SharedPtr sub_bezier_trajectory_;
    void bezier_trajectory_callback(const synapse_msgs::msg::BezierTrajectory& msg) const;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) const;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    void joy_callback(const sensor_msgs::msg::Joy& msg) const;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    void odometry_callback(const nav_msgs::msg::Odometry& msg) const;

    // publications cerebri -> ros
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_actuators_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

    // callbacks
    std::shared_ptr<std::thread> tcp_thread_;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__
