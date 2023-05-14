#include "ros_client.hpp"
#include <actuator_msgs/msg/detail/actuators__struct.hpp>
#include <synapse_tinyframe/SynapseTopics.h>
#include "synapse_protobuf/joy.pb.h"
#include "actuator_msgs/msg/actuators.hpp"


using std::placeholders::_1;

RosClient::RosClient(const std::shared_ptr<TinyFrame> & tf) :
    Node("synapse_ros"),
    tf_(tf) {
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RosClient::joy_callback, this, _1));
    pub_actuators_ = this->create_publisher<actuator_msgs::msg::Actuators>(
            "actuators", 10);
}

void RosClient::publish_actuators(const Actuators & msg) {
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

void RosClient::joy_callback(const sensor_msgs::msg::Joy & msg) const {
    //RCLCPP_INFO(this->get_logger(), "I heard a joy message");

    // translate message from gz to syn
    Joy syn_msg;
    for (auto i = 0u; i < msg.axes.size(); ++i) {
        syn_msg.add_axes(msg.axes[i]);
    }
   
    for (auto i = 0u; i < msg.buttons.size(); ++i) {
        syn_msg.add_buttons(msg.buttons[i]);
    }

    // build tf frame
    TF_Msg frame;
    frame.type = SYNAPSE_IN_JOY_TOPIC;
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Joy" << std::endl;
    }
    frame.len = data.length();
    frame.data = (const uint8_t *)data.c_str();
    tf_send(frame);
}

void RosClient::tf_send(TF_Msg & frame) const {
    TF_Send(tf_.get(), &frame); 
}

// vi: ts=4 sw=4 et
