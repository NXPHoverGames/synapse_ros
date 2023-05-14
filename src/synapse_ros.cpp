#include "clients/ros_client.hpp"
#include "clients/tcp_client.hpp"
#include "synapse_tinyframe/TinyFrame.h"

#include <iostream>
#include <memory>

std::atomic<bool> g_stop{false};
std::shared_ptr<TcpClient> g_tcp_client;
std::shared_ptr<RosClient> g_ros_client;
std::shared_ptr<TinyFrame> g_tf;

void tcp_entry_point()
{
    std::cout << "tcp thread started" << std::endl;
    while (not g_stop) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
    std::cout << "tcp thread stopped" << std::endl;
}

void ros_entry_point()
{
    std::cout << "ros thread started" << std::endl;
    rclcpp::spin(g_ros_client);
    std::cout << "ros thread stopped" << std::endl;
    g_stop = true;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    if (argc < 3) {
        std::cerr << argv[0] << "\thost port" << std::endl;
        return -1;
    }

    // create tinyframe
    g_tf = std::make_shared<TinyFrame>(*(TF_Init(TF_MASTER)));


    // create ros client
    g_ros_client = std::make_shared<RosClient>(g_tf);

    // create tcp client
    g_tcp_client = std::make_shared<TcpClient>(argv[1], std::atoi(argv[2]), g_tf);
    g_tcp_client.get()->ros_ = g_ros_client;
    
    // start threads
    std::thread tcp_thread(tcp_entry_point);
    std::thread ros_thread(ros_entry_point);

    // join threads
    tcp_thread.join();
    ros_thread.join();
    return 0;
}

// vi: ts=4 sw=4 et
