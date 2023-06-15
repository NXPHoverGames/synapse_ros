#include "tcp_client.hpp"
#include "../synapse_ros.hpp"

#include "synapse_protobuf/actuators.pb.h"
#include "synapse_protobuf/twist.pb.h"
#include "synapse_protobuf/joy.pb.h"
#include "synapse_protobuf/odometry.pb.h"

#include "synapse_tinyframe/SynapseTopics.h"
#include <boost/asio/error.hpp>
#include <boost/date_time/posix_time/posix_time_config.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/system/error_code.hpp>
#include <google/protobuf/stubs/logging.h>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

static void write_tcp(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    // get tcp client attached to tf pointer in userdata
    TcpClient * tcp_client = (TcpClient *)tf->userdata;

    // write buffer to tcp client
    tcp_client->write(buf, len);
}

TcpClient::TcpClient(std::string host, int port, const std::shared_ptr<TinyFrame> & tf) : host_(host), port_(port) {
    // Set up the TinyFrame library
    tf_ = tf;
    tf_->usertag = 0;
    tf_->userdata = this;
    tf_->write = write_tcp;
    TF_AddGenericListener(tf_.get(), TcpClient::genericListener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_OUT_CMD_VEL_TOPIC, TcpClient::out_cmd_vel_Listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_OUT_ACTUATORS_TOPIC, TcpClient::actuatorsListener);
    timer_.async_wait(std::bind(&TcpClient::tick, this, _1));
}

void TcpClient::handle_connect(
    const boost::system::error_code& ec,
    const boost::asio::ip::tcp::endpoint& endpoint)
{
    if (ec.failed()) {
        connected_ = false;
        std::cerr << "failed to connect: " << ec.message() << std::endl;
        if (sockfd_.is_open()) {
            sockfd_.close();
        }
    } else {
        connected_ = true;
        std::cout << "tcp connected: " << endpoint << std::endl;
    }
}

void TcpClient::tick(const boost::system::error_code& /*e*/) {
    if (connected_) {
        sockfd_.async_receive(boost::asio::buffer(rx_buf_, rx_buf_length_),
                std::bind(&TcpClient::rx_handler, this, _1, _2));
    } else {
        std::cout << "tcp connecting to: " << host_ << ":" << port_ << std::endl;
        boost::asio::async_connect(
            sockfd_,
            resolver_.resolve(host_, std::to_string(port_)),
            std::bind(&TcpClient::handle_connect, this, _1, _2));
    }

    boost::posix_time::time_duration wait;
    if (connected_) {
        wait = boost::posix_time::milliseconds(100);
    } else {
        wait = boost::posix_time::seconds(5);
    }

    timer_.expires_at(timer_.expires_at() + wait);
    timer_.async_wait(std::bind(&TcpClient::tick, this, _1));
}

void TcpClient::tx_handler(const boost::system::error_code & ec, std::size_t bytes_transferred) {
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

void TcpClient::rx_handler(const boost::system::error_code & ec, std::size_t bytes_transferred) {
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
        connected_ = false;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
        connected_ = false;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "rx error: " << ec.message() << std::endl;
    }
    const std::lock_guard<std::mutex> lock(guard_rx_buf_);
    TF_Accept(tf_.get(), rx_buf_, bytes_transferred);
}

void my_log_handler(google::protobuf::LogLevel level, const char * filename, int line, const std::string & message) {
    static const char* level_names[] = { "INFO", "WARNING", "ERROR", "FATAL" };
    fprintf(stderr, "[libprotobuf %s %s:%d] %s\n",
          level_names[level], filename, line, message.c_str());
    fflush(stderr);  // Needed on MSVC.
}


TF_Result TcpClient::actuatorsListener(TinyFrame *tf, TF_Msg *frame)
{
    // parse protobuf message
    Actuators msg;
    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse actuators" << std::endl;
        return TF_STAY;
    }

    // send to ros
    TcpClient * tcp_client = (TcpClient *)tf->userdata;
    if(tcp_client->ros_ != NULL) {
        tcp_client->ros_->publish_actuators(msg);
    }
    return TF_STAY;
}

TF_Result TcpClient::out_cmd_vel_Listener(TinyFrame *tf, TF_Msg *frame)
{
    (void)tf;
    Twist msg;
    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to out_cmd_vel" << std::endl;
        return TF_STAY;
    } else {
        std::cout << "out cmd vel"
            << msg.linear().x()
            << msg.linear().y()
            << msg.linear().z()
            << msg.angular().x()
            << msg.angular().y()
            << msg.angular().z()
            << std::endl;
    }
    return TF_STAY;
}

TF_Result TcpClient::genericListener(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    int type = msg-> type;
    std::cout << "generic listener id:" << type << std::endl;
    dumpFrameInfo(msg);
    return TF_STAY;
}

void TcpClient::run_for(std::chrono::seconds sec) {
    io_context_.run_for(std::chrono::seconds(sec));
}

void TcpClient::write(const uint8_t *buf, uint32_t len)
{
    if (connected_) {
        boost::asio::async_write(sockfd_, boost::asio::buffer(buf, len), 
                std::bind(&TcpClient::tx_handler, this, _1, _2));
    }
}

// vi: ts=4 sw=4 et
