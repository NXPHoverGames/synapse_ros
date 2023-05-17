#include "synapse_ros.hpp"
#include "clients/tcp_client.hpp"
#include <actuator_msgs/msg/detail/actuators__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <synapse_tinyframe/SynapseTopics.h>

using std::placeholders::_1;
std::shared_ptr<TcpClient> g_tcp_client{NULL};

void tcp_entry_point()
{
    std::cout << "tcp thread started" << std::endl;
    while (rclcpp::ok()) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
    std::cout << "tcp thread stopped" << std::endl;
}

SynapseRos::SynapseRos() :
    Node("synapse_ros") {
    this->declare_parameter("host", "192.0.2.2");
    this->declare_parameter("port", 4242);

    std::string host = this->get_parameter("host").as_string();
    int port = this->get_parameter("port").as_int();

    // subscriptions ros -> cerebri
    sub_actuators_ = this->create_subscription<actuator_msgs::msg::Actuators>(
            "to/actuators", 10, std::bind(&SynapseRos::actuators_callback, this, _1));

    sub_bezier_trajectory_ = this->create_subscription<synapse_msgs::msg::BezierTrajectory>(
            "to/bezier_trajectory", 10, std::bind(&SynapseRos::bezier_trajectory_callback, this, _1));

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "to/cmd_vel", 10, std::bind(&SynapseRos::cmd_vel_callback, this, _1));

    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "to/joy", 10, std::bind(&SynapseRos::joy_callback, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "to/odometry", 10, std::bind(&SynapseRos::odom_callback, this, _1));

    // publications cerebri -> ros
    pub_actuators_ = this->create_publisher<actuator_msgs::msg::Actuators>("from/actuators", 10);
    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("from/odometry", 10);

    // create tcp client
    std::cout << "creating tcp client\n" << std::endl;
    g_tcp_client = std::make_shared<TcpClient>(host, port, tf_);
    g_tcp_client.get()->ros_ = this;
    tcp_thread_ = std::make_shared<std::thread>(tcp_entry_point);
    
}

SynapseRos::~SynapseRos() {
    // join threads
    tcp_thread_->join();
}

void SynapseRos::publish_actuators(const Actuators & msg) {
    (void)msg;

    actuator_msgs::msg::Actuators ros_msg;

    for (auto it=msg.position().begin(); it!=msg.position().end(); it++) {
        ros_msg.position.push_back(*it);
    }

    for (auto it=msg.velocity().begin(); it!=msg.velocity().end(); it++) {
        ros_msg.velocity.push_back(*it);
    }

    for (auto it=msg.normalized().begin(); it!=msg.normalized().end(); it++) {
        ros_msg.normalized.push_back(*it);
    }

    pub_actuators_->publish(ros_msg);
}

void SynapseRos::publish_odometry(const Odometry & msg) {
    (void)msg;

    nav_msgs::msg::Odometry ros_msg;
    pub_odometry_->publish(ros_msg);
}


void SynapseRos::actuators_callback(const actuator_msgs::msg::Actuators & msg) const {
    (void)msg;

    Actuators syn_msg;
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Actuators" << std::endl;
    }
    tf_send(SYNAPSE_IN_ACTUATORS_TOPIC, data);
}

void SynapseRos::bezier_trajectory_callback(const synapse_msgs::msg::BezierTrajectory & msg) const {
    (void)msg;

    BezierTrajectory syn_msg;
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize BezierTrajectory" << std::endl;
    }
    tf_send(SYNAPSE_IN_BEZIER_TRAJECTORY_TOPIC, data);
}

void SynapseRos::cmd_vel_callback(const geometry_msgs::msg::Twist & msg) const {
    (void)msg;

    Twist syn_msg;
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Twist" << std::endl;
    }
    tf_send(SYNAPSE_IN_CMD_VEL_TOPIC, data);
}

void SynapseRos::joy_callback(const sensor_msgs::msg::Joy & msg) const {
    (void)msg;

    Joy syn_msg;
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
    tf_send(SYNAPSE_IN_JOY_TOPIC, data);
}

void SynapseRos::odom_callback(const nav_msgs::msg::Odometry & msg) const {
    (void)msg;

    Odometry syn_msg;
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Odometry" << std::endl;
    }
    tf_send(SYNAPSE_IN_ODOMETRY_TOPIC, data);
}

void SynapseRos::tf_send(int topic, const std::string & data) const {
    TF_Msg frame;
    frame.type = topic;
    frame.len = data.length();
    frame.data = (const uint8_t *)data.c_str();
    TF_Send(tf_.get(), &frame); 
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynapseRos>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
