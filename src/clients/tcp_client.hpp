#ifndef SYNAPSE_ROS_TCP_CLIENT_HPP__
#define SYNAPSE_ROS_TCP_CLIENT_HPP__

#include <algorithm>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/signal_set.hpp>

#include "synapse_tinyframe/TinyFrame.h"
#include "synapse_tinyframe/utils.h"

class SynapseRos;

class TcpClient {
private:
    static const uint32_t rx_buf_length_ = 1024;
    std::mutex guard_rx_buf_;
    uint8_t rx_buf_[rx_buf_length_];
    boost::asio::io_context io_context_ {};
    boost::asio::deadline_timer timer_ { io_context_, boost::posix_time::seconds(0) };
    boost::asio::ip::tcp::socket sockfd_ { boost::asio::ip::tcp::socket(io_context_) };
    boost::asio::ip::tcp::resolver resolver_ { io_context_ };
    std::shared_ptr<TinyFrame> tf_ { TF_Init(TF_MASTER) };
    std::string host_;
    int port_;
    bool connected_ { false };

public:
    SynapseRos* ros_ { NULL };
    TcpClient(std::string host, int port, const std::shared_ptr<TinyFrame>& tf);
    void run_for(std::chrono::seconds sec);
    void write(const uint8_t* buf, uint32_t len);

private:
    void handle_connect(
        const boost::system::error_code& ec,
        const boost::asio::ip::tcp::endpoint& endpoint);
    void tick(const boost::system::error_code& /*e*/);
    void tx_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void rx_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void send_frame(TF_Msg* msg);
    static TF_Result odometryListener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result actuatorsListener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result out_cmd_vel_Listener(TinyFrame* tf, TF_Msg* frame);
    static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg);
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_TCP_CLIENT_HPP__
