#include "synapse_ros.hpp"
#include "clients/tcp_client.hpp"
#include <synapse_msgs/msg/detail/led_array__struct.hpp>
#include <synapse_msgs/msg/detail/safety__struct.hpp>
#include <synapse_protobuf/led.pb.h>
#include <synapse_protobuf/safety.pb.h>
#include <synapse_tinyframe/SynapseTopics.h>

using std::placeholders::_1;
std::shared_ptr<TcpClient> g_tcp_client { NULL };

void tcp_entry_point()
{
    while (rclcpp::ok()) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
}

SynapseRos::SynapseRos()
    : Node("synapse_ros")
{
    this->declare_parameter("host", "192.0.2.1");
    this->declare_parameter("port", 4242);

    std::string host = this->get_parameter("host").as_string();
    int port = this->get_parameter("port").as_int();

    // subscriptions ros -> cerebri
    sub_actuators_ = this->create_subscription<actuator_msgs::msg::Actuators>(
        "in/actuators", 10, std::bind(&SynapseRos::actuators_callback, this, _1));

    sub_bezier_trajectory_ = this->create_subscription<synapse_msgs::msg::BezierTrajectory>(
        "in/bezier_trajectory", 10, std::bind(&SynapseRos::bezier_trajectory_callback, this, _1));

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "in/cmd_vel", 10, std::bind(&SynapseRos::cmd_vel_callback, this, _1));

    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "in/joy", 10, std::bind(&SynapseRos::joy_callback, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "in/odometry", 10, std::bind(&SynapseRos::odometry_callback, this, _1));

    sub_led_array_ = this->create_subscription<synapse_msgs::msg::LEDArray>(
        "in/led_array", 10, std::bind(&SynapseRos::led_array_callback, this, _1));

    // publications cerebri -> ros
    pub_actuators_ = this->create_publisher<actuator_msgs::msg::Actuators>("out/actuators", 10);
    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("out/odometry", 10);
    pub_battery_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>("out/battery_state", 10);
    pub_fsm_ = this->create_publisher<synapse_msgs::msg::FSM>("out/fsm", 10);
    pub_safety_ = this->create_publisher<synapse_msgs::msg::Safety>("out/safety", 10);
    pub_uptime_ = this->create_publisher<builtin_interfaces::msg::Time>("out/uptime", 10);
    pub_clock_offset_ = this->create_publisher<builtin_interfaces::msg::Time>("out/uptime", 10);

    // create tcp client
    g_tcp_client = std::make_shared<TcpClient>(host, port);
    g_tcp_client.get()->ros_ = this;
    tf_ = g_tcp_client.get()->tf_;
    tcp_thread_ = std::make_shared<std::thread>(tcp_entry_point);
}

SynapseRos::~SynapseRos()
{
    // join threads
    tcp_thread_->join();
}

void SynapseRos::publish_actuators(const synapse::msgs::Actuators& msg)
{
    actuator_msgs::msg::Actuators ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header.frame_id = msg.header().frame_id();
        if (msg.header().has_stamp()) {
            ros_msg.header.stamp.sec = msg.header().stamp().sec();
            ros_msg.header.stamp.nanosec = msg.header().stamp().nanosec();
        }
    }

    // actuators
    for (auto it = msg.position().begin(); it != msg.position().end(); it++) {
        ros_msg.position.push_back(*it);
    }

    for (auto it = msg.velocity().begin(); it != msg.velocity().end(); it++) {
        ros_msg.velocity.push_back(*it);
    }

    for (auto it = msg.normalized().begin(); it != msg.normalized().end(); it++) {
        ros_msg.normalized.push_back(*it);
    }

    pub_actuators_->publish(ros_msg);
}

void SynapseRos::publish_odometry(const synapse::msgs::Odometry& msg)
{
    nav_msgs::msg::Odometry ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header.frame_id = msg.header().frame_id();
        if (msg.header().has_stamp()) {
            ros_msg.header.stamp.sec = msg.header().stamp().sec();
            ros_msg.header.stamp.nanosec = msg.header().stamp().nanosec();
        }
    }

    // child frame id
    ros_msg.child_frame_id = msg.child_frame_id();

    // pose
    ros_msg.pose.pose.position.x = msg.pose().pose().position().x();
    ros_msg.pose.pose.position.y = msg.pose().pose().position().y();
    ros_msg.pose.pose.position.z = msg.pose().pose().position().z();
    ros_msg.pose.pose.orientation.x = msg.pose().pose().orientation().x();
    ros_msg.pose.pose.orientation.y = msg.pose().pose().orientation().y();
    ros_msg.pose.pose.orientation.z = msg.pose().pose().orientation().z();
    ros_msg.pose.pose.orientation.w = msg.pose().pose().orientation().w();

    // twist
    ros_msg.twist.twist.linear.x = msg.twist().twist().linear().x();
    ros_msg.twist.twist.linear.y = msg.twist().twist().linear().y();
    ros_msg.twist.twist.linear.z = msg.twist().twist().linear().z();
    ros_msg.twist.twist.angular.x = msg.twist().twist().angular().x();
    ros_msg.twist.twist.angular.y = msg.twist().twist().angular().y();
    ros_msg.twist.twist.angular.z = msg.twist().twist().angular().z();

    pub_odometry_->publish(ros_msg);
}

void SynapseRos::publish_battery_state(const synapse::msgs::BatteryState& msg)
{
    sensor_msgs::msg::BatteryState ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header.frame_id = msg.header().frame_id();
        if (msg.header().has_stamp()) {
            ros_msg.header.stamp.sec = msg.header().stamp().sec();
            ros_msg.header.stamp.nanosec = msg.header().stamp().nanosec();
        }
    }

    ros_msg.voltage = msg.voltage();
    pub_battery_state_->publish(ros_msg);
}

void SynapseRos::publish_fsm(const synapse::msgs::Fsm& msg)
{
    synapse_msgs::msg::FSM ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header.frame_id = msg.header().frame_id();
        if (msg.header().has_stamp()) {
            ros_msg.header.stamp.sec = msg.header().stamp().sec();
            ros_msg.header.stamp.nanosec = msg.header().stamp().nanosec();
        }
    }

    ros_msg.mode = msg.mode();
    ros_msg.armed = msg.armed();

    pub_fsm_->publish(ros_msg);
}

void SynapseRos::publish_safety(const synapse::msgs::Safety& msg)
{
    synapse_msgs::msg::Safety ros_msg;

    // header
    if (msg.has_header()) {
        ros_msg.header.frame_id = msg.header().frame_id();
        if (msg.header().has_stamp()) {
            ros_msg.header.stamp.sec = msg.header().stamp().sec();
            ros_msg.header.stamp.nanosec = msg.header().stamp().nanosec();
        }
    }

    ros_msg.status = msg.status();

    pub_safety_->publish(ros_msg);
}

void SynapseRos::publish_uptime(const synapse::msgs::Time& msg)
{
    builtin_interfaces::msg::Time ros_uptime;

    rclcpp::Time::now();

    ros_uptime.sec = msg.sec();
    ros_uptime.nanosec = msg.nanosec();

    ros_clock_offset.sec = msg.sec();
    ros_clock_offset.nanosec = msg.nanosec();

    pub_uptime_->publish(ros_uptime);
    pub_clock_offset_->publish(ros_clock_offset_);
}

void SynapseRos::actuators_callback(const actuator_msgs::msg::Actuators& msg) const
{
    synapse::msgs::Actuators syn_msg;

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    // actuators
    for (auto i = 0u; i < msg.position.size(); ++i) {
        syn_msg.add_position(msg.position[i]);
    }

    for (auto i = 0u; i < msg.velocity.size(); ++i) {
        syn_msg.add_velocity(msg.velocity[i]);
    }
    for (auto i = 0u; i < msg.normalized.size(); ++i) {
        syn_msg.add_normalized(msg.normalized[i]);
    }

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Actuators" << std::endl;
    }
    tf_send(SYNAPSE_ACTUATORS_TOPIC, data);
}

void SynapseRos::bezier_trajectory_callback(const synapse_msgs::msg::BezierTrajectory& msg) const
{
    synapse::msgs::BezierTrajectory syn_msg;

    syn_msg.set_time_start(msg.time_start);

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    for (auto i = 0u; i < msg.curves.size(); ++i) {
        synapse::msgs::BezierCurve* curve = syn_msg.add_curves();

        curve->set_time_stop(msg.curves[i].time_stop);

        for (auto j = 0u; j < msg.curves[i].x.size(); ++j) {
            curve->add_x(msg.curves[i].x[j]);
        }

        for (auto j = 0u; j < msg.curves[i].y.size(); ++j) {
            curve->add_y(msg.curves[i].y[j]);
        }

        for (auto j = 0u; j < msg.curves[i].z.size(); ++j) {
            curve->add_z(msg.curves[i].z[j]);
        }

        for (auto j = 0u; j < msg.curves[i].yaw.size(); ++j) {
            curve->add_yaw(msg.curves[i].yaw[j]);
        }
    }

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize BezierTrajectory" << std::endl;
    }
    tf_send(SYNAPSE_BEZIER_TRAJECTORY_TOPIC, data);
}

void SynapseRos::cmd_vel_callback(const geometry_msgs::msg::Twist& msg) const
{
    synapse::msgs::Twist syn_msg;

    // twist
    syn_msg.mutable_linear()->set_x(msg.linear.x);
    syn_msg.mutable_linear()->set_y(msg.linear.y);
    syn_msg.mutable_linear()->set_z(msg.linear.z);
    syn_msg.mutable_angular()->set_x(msg.angular.x);
    syn_msg.mutable_angular()->set_y(msg.angular.y);
    syn_msg.mutable_angular()->set_z(msg.angular.z);

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Twist" << std::endl;
    }
    tf_send(SYNAPSE_CMD_VEL_TOPIC, data);
}

void SynapseRos::joy_callback(const sensor_msgs::msg::Joy& msg) const
{
    synapse::msgs::Joy syn_msg;
    for (auto i = 0u; i < msg.axes.size(); ++i) {
        syn_msg.add_axes(msg.axes[i]);
    }

    for (auto i = 0u; i < msg.buttons.size(); ++i) {
        syn_msg.add_buttons(msg.buttons[i]);
    }

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Joy" << std::endl;
    }
    tf_send(SYNAPSE_JOY_TOPIC, data);
}

void SynapseRos::odometry_callback(const nav_msgs::msg::Odometry& msg) const
{
    // construct empty syn_msg
    synapse::msgs::Odometry syn_msg {};

    // child frame
    syn_msg.set_child_frame_id(msg.child_frame_id);

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    // pose
    syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_x(msg.pose.pose.position.x);
    syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_y(msg.pose.pose.position.y);
    syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_z(msg.pose.pose.position.z);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(msg.pose.pose.orientation.x);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(msg.pose.pose.orientation.y);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(msg.pose.pose.orientation.z);
    syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(msg.pose.pose.orientation.w);
    // skipping covariance

    // twist
    syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_x(msg.twist.twist.linear.x);
    syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_y(msg.twist.twist.linear.y);
    syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_z(msg.twist.twist.linear.z);
    syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_x(msg.twist.twist.angular.x);
    syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_y(msg.twist.twist.angular.y);
    syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_z(msg.twist.twist.angular.z);
    // skipping covariance

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Odometry" << std::endl;
    }
    tf_send(SYNAPSE_ODOMETRY_TOPIC, data);
}

void SynapseRos::led_array_callback(const synapse_msgs::msg::LEDArray& msg) const
{
    // construct empty syn_msg
    synapse::msgs::LEDArray syn_msg {};

    // header
    syn_msg.mutable_header()->set_frame_id(msg.header.frame_id);
    syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header.stamp.sec);
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header.stamp.nanosec);

    // leds
    for (auto i = 0u; i < msg.led.size(); ++i) {
        synapse::msgs::LED* led = syn_msg.add_led();
        led->set_index(msg.led[i].index);
        led->set_b(msg.led[i].b);
        led->set_g(msg.led[i].g);
        led->set_r(msg.led[i].r);
    }

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize LEDArray" << std::endl;
    }
    tf_send(SYNAPSE_LED_ARRAY_TOPIC, data);
}

void SynapseRos::clock_offset_callback(const builtin_interfaces::msg::Time& msg) const
{
    // construct empty syn_msg
    synapse::msgs::Time syn_msg {};

    syn_msg.set_sec(msg.sec);
    syn_msg.set_nanosec(msg.nanosec);

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Clock Offset" << std::endl;
    }
    tf_send(SYNAPSE_CLOCK_OFFSET_TOPIC, data);
}

void SynapseRos::tf_send(int topic, const std::string& data) const
{
    TF_Msg frame;
    frame.type = topic;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    TF_Send(tf_.get(), &frame);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynapseRos>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
