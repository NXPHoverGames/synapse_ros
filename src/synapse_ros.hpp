#ifndef SYNAPSE_ROS_CLIENT_HPP__
#define SYNAPSE_ROS_CLIENT_HPP__

#include "rclcpp/rclcpp.hpp"

#include <memory>

#include "synapse_tinyframe/TinyFrame.h"
#include <rclcpp/subscription_options.hpp>
#include <synapse_msgs/msg/detail/led_array__struct.hpp>

#include "actuator_msgs/msg/actuators.hpp"
#include "synapse_protobuf/actuators.pb.h"

#include "nav_msgs/msg/odometry.hpp"
#include "synapse_protobuf/odometry.pb.h"

#include "sensor_msgs/msg/joy.hpp"
#include "synapse_protobuf/joy.pb.h"

#include "synapse_msgs/msg/bezier_trajectory.hpp"
#include "synapse_protobuf/bezier_trajectory.pb.h"

#include "synapse_msgs/msg/led_array.hpp"
#include "synapse_protobuf/led_array.pb.h"

#include "synapse_msgs/msg/fsm.hpp"
#include "synapse_protobuf/fsm.pb.h"

#include "sensor_msgs/msg/battery_state.hpp"
#include "synapse_protobuf/battery_state.pb.h"

#include "synapse_msgs/msg/safety.hpp"
#include "synapse_protobuf/safety.pb.h"

#include "geometry_msgs/msg/twist.hpp"
#include "synapse_protobuf/twist.pb.h"

#include "builtin_interfaces/msg/time.hpp"
#include "synapse_protobuf/time.pb.h"

class TcpClient;

void tcp_entry_point();

class SynapseRos : public rclcpp::Node {
public:
    SynapseRos();
    virtual ~SynapseRos();
    void tf_send(int topic, const std::string& data) const;
    void publish_actuators(const synapse::msgs::Actuators& msg);
    void publish_odometry(const synapse::msgs::Odometry& msg);
    void publish_battery_state(const synapse::msgs::BatteryState& msg);
    void publish_fsm(const synapse::msgs::Fsm& msg);
    void publish_safety(const synapse::msgs::Safety& msg);
    void publish_uptime(const synapse::msgs::Time& msg);

private:
    std::shared_ptr<TinyFrame> tf_ {};
    builtin_interfaces::msg::Time ros_clock_offset_;

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

    rclcpp::Subscription<synapse_msgs::msg::LEDArray>::SharedPtr sub_led_array_;
    void led_array_callback(const synapse_msgs::msg::LEDArray& msg) const;

    // publications cerebri -> ros
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_actuators_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
    rclcpp::Publisher<synapse_msgs::msg::FSM>::SharedPtr pub_fsm_;
    rclcpp::Publisher<synapse_msgs::msg::Safety>::SharedPtr pub_safety_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_uptime_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_clock_offset_;

    // callbacks
    std::shared_ptr<std::thread> tcp_thread_;
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_CLIENT_HPP__
