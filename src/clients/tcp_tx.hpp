#ifndef SYNAPSE_ROS_TCP_CLIENT_HPP__
#define SYNAPSE_ROS_TCP_CLIENT_HPP__

#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/signal_set.hpp>

#include "synapse_tinyframe/TinyFrame.h"

class Tx;

class TcpTx {
private:
    boost::asio::io_context io_context_ {};
    boost::asio::deadline_timer timer_ { io_context_, boost::posix_time::seconds(0) };
    boost::asio::ip::tcp::socket sockfd_ { boost::asio::ip::tcp::socket(io_context_) };
    boost::asio::ip::tcp::resolver resolver_ { io_context_ };
    std::string host_;
    int port_;
    bool connected_ { false };

public:
    std::shared_ptr<TinyFrame> tf_ {};
    Tx* ros_ { NULL };
    TcpTx(std::string host, int port);
    void run_for(std::chrono::seconds sec);
    void write(const uint8_t* buf, uint32_t len);

private:
    void handle_connect(
        const boost::system::error_code& ec,
        const boost::asio::ip::tcp::endpoint& endpoint);
    void tick(const boost::system::error_code& /*e*/);
    void tx_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void send_frame(TF_Msg* msg);
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_ROS_TCP_CLIENT_HPP__
