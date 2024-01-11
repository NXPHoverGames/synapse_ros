#ifndef SYNAPSE_ROS_PROTO_UDP_LINK_HPP__
#define SYNAPSE_ROS_PROTO_UDP_LINK_HPP__

#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/date_time/posix_time/posix_time_config.hpp>

#include "synapse_tinyframe/TinyFrame.h"

class SynapseRos;

class UDPLink {
private:
    static const uint32_t rx_buf_length_ = 1024;
    std::mutex guard_rx_buf_;
    uint8_t rx_buf_[rx_buf_length_];
    boost::asio::io_context io_context_ {};
    boost::asio::ip::udp::socket sock_ {
        boost::asio::ip::udp::socket(io_context_,
            boost::asio::ip::udp::endpoint(
                boost::asio::ip::udp::v4(), 4242))
    };
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::asio::ip::udp::endpoint my_endpoint_;

public:
    std::shared_ptr<TinyFrame> tf_ {};
    SynapseRos* ros_ { NULL };
    UDPLink(std::string host, int port);
    void run_for(std::chrono::seconds sec);
    void write(const uint8_t* buf, uint32_t len);

private:
    void timeout_handler();
    void tx_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void rx_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void send_frame(TF_Msg* msg);
    static TF_Result odometry_listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result actuators_listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result out_cmd_vel_listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result battery_state_listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result nav_sat_fix_listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result status_listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result uptime_listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result generic_listener(TinyFrame* tf, TF_Msg* msg);
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_PROTO_UDP_LINK_HPP__
