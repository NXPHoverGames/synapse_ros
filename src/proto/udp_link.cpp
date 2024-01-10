#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/utils.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/twist.pb.h>

#include <boost/asio/error.hpp>
#include <boost/system/error_code.hpp>

#include "../synapse_ros.hpp"
#include "udp_link.hpp"

using boost::asio::ip::udp;
using std::placeholders::_1;
using std::placeholders::_2;

static void write_udp(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    // get udp link attached to tf pointer in userdata
    UDPLink* udp_link = (UDPLink*)tf->userdata;

    // write buffer to udp link
    udp_link->write(buf, len);
}

UDPLink::UDPLink(std::string host, int port)
{
    remote_endpoint_ = *udp::resolver(io_context_).resolve(udp::resolver::query(host, std::to_string(port)));
    my_endpoint_ = udp::endpoint(udp::v4(), 4242);

    // Set up the TinyFrame library
    tf_ = std::make_shared<TinyFrame>(*TF_Init(TF_MASTER, write_udp));
    tf_->usertag = 0;
    tf_->userdata = this;
    tf_->write = write_udp;
    TF_AddGenericListener(tf_.get(), UDPLink::generic_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_CMD_VEL_TOPIC, UDPLink::out_cmd_vel_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_ACTUATORS_TOPIC, UDPLink::actuators_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_ODOMETRY_TOPIC, UDPLink::odometry_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_BATTERY_STATE_TOPIC, UDPLink::battery_state_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_NAV_SAT_FIX_TOPIC, UDPLink::nav_sat_fix_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_STATUS_TOPIC, UDPLink::status_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_UPTIME_TOPIC, UDPLink::uptime_listener);

    // schedule new rx
    sock_.async_receive_from(boost::asio::buffer(rx_buf_, rx_buf_length_),
        my_endpoint_,
        std::bind(&UDPLink::rx_handler, this, _1, _2));
}

void UDPLink::tx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    (void)bytes_transferred;
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "tx error: " << ec.message() << std::endl;
    }
}

void UDPLink::rx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "rx error: " << ec.message() << std::endl;
    } else if (ec == boost::system::errc::success) {
        const std::lock_guard<std::mutex> lock(guard_rx_buf_);
        TF_Accept(tf_.get(), rx_buf_, bytes_transferred);
    }

    // schedule new rx
    sock_.async_receive_from(boost::asio::buffer(rx_buf_, rx_buf_length_),
        my_endpoint_,
        std::bind(&UDPLink::rx_handler, this, _1, _2));
}

TF_Result UDPLink::actuators_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse::msgs::Actuators msg;

    // get udp link attached to tf pointer in userdata
    UDPLink* udp_link = (UDPLink*)tf->userdata;

    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse actuators" << std::endl;
        return TF_STAY;
    }

    // send to ros
    if (udp_link->ros_ != NULL) {
        udp_link->ros_->publish_actuators(msg);
    }
    return TF_STAY;
}

TF_Result UDPLink::odometry_listener(TinyFrame* tf, TF_Msg* frame)
{
    // parse protobuf message
    synapse::msgs::Odometry syn_msg;
    if (!syn_msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse odometry" << std::endl;
        return TF_STAY;
    }

    // send to ros
    UDPLink* udp_link = (UDPLink*)tf->userdata;
    if (udp_link->ros_ != NULL) {
        udp_link->ros_->publish_odometry(syn_msg);
    }
    return TF_STAY;
}

TF_Result UDPLink::battery_state_listener(TinyFrame* tf, TF_Msg* frame)
{
    // parse protobuf message
    synapse::msgs::BatteryState syn_msg;
    if (!syn_msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse battery state" << std::endl;
        return TF_STAY;
    }

    // send to ros
    UDPLink* udp_link = (UDPLink*)tf->userdata;
    if (udp_link->ros_ != NULL) {
        udp_link->ros_->publish_battery_state(syn_msg);
    }
    return TF_STAY;
}

TF_Result UDPLink::nav_sat_fix_listener(TinyFrame* tf, TF_Msg* frame)
{
    // parse protobuf message
    synapse::msgs::NavSatFix syn_msg;
    if (!syn_msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse battery state" << std::endl;
        return TF_STAY;
    }

    // send to ros
    UDPLink* udp_link = (UDPLink*)tf->userdata;
    if (udp_link->ros_ != NULL) {
        udp_link->ros_->publish_nav_sat_fix(syn_msg);
    }
    return TF_STAY;
}

TF_Result UDPLink::status_listener(TinyFrame* tf, TF_Msg* frame)
{
    // parse protobuf message
    synapse::msgs::Status syn_msg;
    if (!syn_msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse status" << std::endl;
        return TF_STAY;
    }

    // send to ros
    UDPLink* udp_link = (UDPLink*)tf->userdata;
    if (udp_link->ros_ != NULL) {
        udp_link->ros_->publish_status(syn_msg);
    }
    return TF_STAY;
}

TF_Result UDPLink::out_cmd_vel_listener(TinyFrame* tf, TF_Msg* frame)
{
    (void)tf;
    synapse::msgs::Twist msg;
    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to out_cmd_vel" << std::endl;
        return TF_STAY;
    } else {
    }
    return TF_STAY;
}

TF_Result UDPLink::uptime_listener(TinyFrame* tf, TF_Msg* frame)
{
    // parse protobuf message
    synapse::msgs::Time syn_msg;
    if (!syn_msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse uptime" << std::endl;
        return TF_STAY;
    }

    // send to ros
    UDPLink* udp_link = (UDPLink*)tf->userdata;
    if (udp_link->ros_ != NULL) {
        udp_link->ros_->publish_uptime(syn_msg);
    }
    return TF_STAY;
}

TF_Result UDPLink::generic_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    int type = msg->type;
    std::cout << "generic listener id:" << type << std::endl;
    dumpFrameInfo(msg);
    return TF_STAY;
}

void UDPLink::run_for(std::chrono::seconds sec)
{
    io_context_.run_for(std::chrono::seconds(sec));
}

void UDPLink::write(const uint8_t* buf, uint32_t len)
{
    sock_.async_send_to(boost::asio::buffer(buf, len),
        remote_endpoint_,
        std::bind(&UDPLink::tx_handler, this, _1, _2));
}

// vi: ts=4 sw=4 et
