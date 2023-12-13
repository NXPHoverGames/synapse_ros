#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/utils.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/twist.pb.h>

#include <boost/asio/error.hpp>
#include <boost/date_time/posix_time/posix_time_config.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/system/error_code.hpp>

#include "../tx.hpp"
#include "tcp_tx.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

static void write_tcp(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    // get tcp client attached to tf pointer in userdata
    TcpTx* tcp_client = (TcpTx*)tf->userdata;

    // write buffer to tcp client
    tcp_client->write(buf, len);
}

TcpTx::TcpTx(std::string host, int port)
    : host_(host)
    , port_(port)
{
    // Set socket options
    sockfd_.open(boost::asio::ip::tcp::v4());
    if (!sockfd_.is_open()) {
        std::cerr << "failed to open socket" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_REUSEADDR> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set reuseaddr" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_KEEPALIVE> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keep alive" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_KEEPIDLE> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keepidle" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_KEEPCNT> { 3 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keepcnt" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_KEEPINTVL> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set keepintvl" << std::endl;
    }

    try {
        sockfd_.set_option(boost::asio::detail::socket_option::integer<IPPROTO_TCP, TCP_SYNCNT> { 1 });
    } catch (std::exception& e) {
        std::cerr << e.what() << "failed to set user syncnt" << std::endl;
    }

    // Set up the TinyFrame library
    tf_ = std::make_shared<TinyFrame>(*TF_Init(TF_MASTER, write_tcp));
    tf_->usertag = 0;
    tf_->userdata = this;
    tf_->write = write_tcp;
    timer_.async_wait(std::bind(&TcpTx::tick, this, _1));
}

void TcpTx::handle_connect(
    const boost::system::error_code& ec,
    const boost::asio::ip::tcp::endpoint& endpoint)
{
    if (ec.failed()) {
        connected_ = false;
        if (sockfd_.is_open()) {
            sockfd_.close();
        }
    } else {
        connected_ = true;
        std::cout << "tcp connected: " << endpoint << std::endl;
    }
}

void TcpTx::tick(const boost::system::error_code& /*e*/)
{
    if (!connected_) {
        boost::asio::async_connect(
            sockfd_,
            resolver_.resolve(host_, std::to_string(port_)),
            std::bind(&TcpTx::handle_connect, this, _1, _2));
    }
    boost::posix_time::time_duration wait;
    wait = boost::posix_time::milliseconds(100);
    timer_.expires_at(timer_.expires_at() + wait);
    timer_.async_wait(std::bind(&TcpTx::tick, this, _1));
}

void TcpTx::tx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    (void)bytes_transferred;
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
        connected_ = false;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
        connected_ = false;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "tx error: " << ec.message() << std::endl;
    }
}

void TcpTx::run_for(std::chrono::seconds sec)
{
    io_context_.run_for(std::chrono::seconds(sec));
}

void TcpTx::write(const uint8_t* buf, uint32_t len)
{
    if (connected_) {
        boost::asio::async_write(sockfd_, boost::asio::buffer(buf, len),
            std::bind(&TcpTx::tx_handler, this, _1, _2));
    }
}

// vi: ts=4 sw=4 et
