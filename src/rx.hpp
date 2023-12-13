#ifndef SYNAPSE_ROS_CLIENT_HPP__
#define SYNAPSE_ROS_CLIENT_HPP__

#include <actuator_msgs/msg/actuators.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <synapse_msgs/msg/bezier_trajectory.hpp>
#include <synapse_msgs/msg/status.hpp>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/battery_state.pb.h>
#include <synapse_protobuf/bezier_trajectory.pb.h>
#include <synapse_protobuf/header.pb.h>
#include <synapse_protobuf/imu.pb.h>
#include <synapse_protobuf/joy.pb.h>
#include <synapse_protobuf/magnetic_field.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/status.pb.h>
#include <synapse_protobuf/time.pb.h>
#include <synapse_protobuf/twist.pb.h>
#include <synapse_protobuf/wheel_odometry.pb.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>

class TcpRx;

void tcp_entry_point();

class Rx : public rclcpp::Node {
public:
    Rx();
    virtual ~Rx();
    void tf_send(int topic, const std::string& data) const;
    void publish_actuators(const synapse::msgs::Actuators& msg);
    void publish_odometry(const synapse::msgs::Odometry& msg);
    void publish_battery_state(const synapse::msgs::BatteryState& msg);
    void publish_status(const synapse::msgs::Status& msg);
    void publish_uptime(const synapse::msgs::Time& msg);

private:
    std::shared_ptr<TinyFrame> tf_ {};
    builtin_interfaces::msg::Time ros_clock_offset_ {};

    std_msgs::msg::Header compute_header(const synapse::msgs::Header& msg);

    // publications cerebri -> ros
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_actuators_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
    rclcpp::Publisher<synapse_msgs::msg::Status>::SharedPtr pub_status_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_uptime_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_clock_offset_;

    // callbacks
    std::shared_ptr<std::thread> tcp_thread_;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__
