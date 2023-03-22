//
// blocking_tcp_echo_client.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2021 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/asio.hpp>

#include <synapse_protobuf/twist.pb.h>

#define TX_BUF_SIZE 100
#define RX_BUF_SIZE 100

using namespace std;
using boost::asio::ip::tcp;

int main(int argc, char* argv[])
{
  if (argc != 3)
  {
    cerr << "Usage: blocking_tcp_echo_client <host> <port>\n";
    return 1;
  }

  char tx_buf[TX_BUF_SIZE];
  char rx_buf[RX_BUF_SIZE];

  // connecto tcp
  boost::asio::io_context io_context;
  tcp::socket s(io_context);
  tcp::resolver resolver(io_context);
  boost::asio::connect(s, resolver.resolve(argv[1], argv[2]));

  while (true) {

    // send twist
    {
      Twist twist_send;
      fstream output(tx_buf);
      if (!twist_send.SerializeToOstream(&output)) {
        cerr << "Failed to write twist to tx_buf." << endl;
        continue;
      }
      boost::asio::write(s, boost::asio::buffer(tx_buf, TX_BUF_SIZE));
    }

    // receive twist
    {
      Twist twist_reply;
      boost::asio::read(s, boost::asio::buffer(rx_buf, RX_BUF_SIZE));

      fstream input(rx_buf);
      if (!twist_reply.ParseFromIstream(&input)) {
        cerr << "Failed to parse twist." << endl;
        return -1;
      }
      cout << "Reply is: ";
      cout << twist_reply.linear().x() << twist_reply.linear().y()
        << twist_reply.linear().z()
        << twist_reply.angular().x() << twist_reply.angular().y()
        << twist_reply.angular().z() << endl;

    }

  }

  return 0;
}
