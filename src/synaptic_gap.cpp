/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h> 

#include <pthread.h>

#include "synapse_protobuf/twist.pb.h"
#include "synapse_protobuf/joy.pb.h"
#include "synapse_protobuf/odometry.pb.h"
#include "synapse_tinyframe/TinyFrame.h"
#include "synapse_tinyframe/utils.h"
#include "synapse_tinyframe/SynapseTopics.h"

#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

#define RX_BUF_SIZE 1024

#define BIND_PORT 4242
#define CONFIG_SYNAPTIC_GAP_ETHERNET
//#define CONFIG_SYNAPTIC_GAP_UART

using namespace std;

static int sockfd = 0;

/**
 * This function should be defined in the application code.
 * It implements the lowest layer - sending bytes to UART (or other)
 */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len)
{
#if defined(CONFIG_SYNAPTIC_GAP_UART)
  // uart
  if (tf->usertag == 0) {
    for (int i=0; i<len; i++) {
      uart_poll_out(uart_dev, buf[i]);
    }
  }
#endif

#if defined(CONFIG_SYNAPTIC_GAP_ETHERNET)
  // ethernet
  if (tf->usertag == 1) {
    int out_len;
    const uint8_t *p;
    p = buf;
    do {
      out_len = send(sockfd, p, len, 0);
      if (out_len < 0) {
        printf("error: send: %d\n", errno);
        return;
      }
      p += out_len;
      len -= out_len;
    } while (len);
  }
#endif
}

static TF_Result genericListener(TinyFrame *tf, TF_Msg *msg)
{
  (void)tf;
  dumpFrameInfo(msg);
  return TF_STAY;
}

static TF_Result cmdVelListener(TinyFrame *tf, TF_Msg *msg)
{
  (void)tf;

  /* Allocate space for the decoded message. */
  Twist message;

  /* Create a stream that reads from the buffer. */
  string data((char *)msg->data, msg->len);
  if (!message.ParseFromString(data)) {
    cerr << "Failed to parse cmd vel." << endl;
    return TF_STAY;
  }

  /* Print the data contained in the message. */
  printf("%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n", 
    message.linear().x(), message.linear().y(), message.linear().z(),
    message.angular().x(), message.angular().y(), message.angular().z());

  return TF_STAY;
}

static TF_Result joyListener(TinyFrame *tf, TF_Msg *msg)
{
  (void)tf;

  /* Allocate space for the decoded message. */
  Joy message;

  /* Create a stream that reads from the buffer. */
  string data((char *)msg->data, msg->len);
  if (!message.ParseFromString(data)) {
    cerr << "Failed to parse joy." << endl;
    return TF_STAY;
  }

  return TF_STAY;
}

static TF_Result odometryExternalListener(TinyFrame *tf, TF_Msg *msg)
{
  (void)tf;

  /* Allocate space for the decoded message. */
  Odometry message;

  /* Create a stream that reads from the buffer. */
  string data((char *)msg->data, msg->len);
  if (!message.ParseFromString(data)) {
    cerr << "Failed to parse odometry." << endl;
    return TF_STAY;
  }

  return TF_STAY;
}

#if defined(CONFIG_SYNAPTIC_GAP_UART)
void * uart_entry_point(void *)
{
  TF_Msg msg;

  // Set up the TinyFrame library
  TinyFrame *tf0;
  tf0 = TF_Init(TF_MASTER); // 1 = master, 0 = slave
  tf0->usertag = 0;
  TF_AddGenericListener(tf0, genericListener);
  TF_AddTypeListener(tf0, SYNAPSE_IN_CMD_VEL_TOPIC, cmdVelListener);
  TF_AddTypeListener(tf0, SYNAPSE_IN_JOY_TOPIC, joyListener);
  TF_AddTypeListener(tf0, SYNAPSE_IN_ODOMETRY_TOPIC, odometryExternalListener);

  while (true) {
    // send cmd vel message
    {
      Twist message;
      message.mutable_linear()->set_x(1);
      message.mutable_linear()->set_y(2);
      message.mutable_linear()->set_z(3);
      message.mutable_angular()->set_x(4);
      message.mutable_angular()->set_y(5);
      message.mutable_angular()->set_z(6);

      string data;
      if (!message.SerializeToString(&data)) {
        cerr << "Failed to write cmd vel." << endl;
      } else {
        TF_ClearMsg(&msg);
        msg.type = SYNAPSE_IN_CMD_VEL_TOPIC;
        msg.len = data.length();
        msg.data = (const uint8_t *)data.c_str();
        TF_Send(tf0, &msg);
      }
    }

    // receive messages
    {
      uint8_t c;
      int count = 0;
      while (uart_poll_in(uart_dev, &c) == 0) {
        TF_AcceptChar(tf0, c);
        count++;
      }
    }

    // should move TF tick to a clock thread
    TF_Tick(tf0);
  }
}
#endif

#if defined(CONFIG_SYNAPTIC_GAP_ETHERNET)
void * ethernet_entry_point(void *)
{

  struct sockaddr_in servaddr;
  
  sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (sockfd < 0) {
    printf("error: socket: %d\n", errno);
    exit(1);
  }

  const char * serv_ip = "192.0.2.1";

  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(serv_ip);
  servaddr.sin_port = htons(BIND_PORT);

  TF_Msg msg;

  // Set up the TinyFrame library
  static TinyFrame *tf0;
  tf0 = TF_Init(TF_MASTER); // 1 = master, 0 = slave
  tf0->usertag = 1;
  TF_AddGenericListener(tf0, genericListener);
  TF_AddTypeListener(tf0, SYNAPSE_IN_CMD_VEL_TOPIC, cmdVelListener);
  TF_AddTypeListener(tf0, SYNAPSE_IN_JOY_TOPIC, joyListener);
  TF_AddTypeListener(tf0, SYNAPSE_IN_ODOMETRY_TOPIC, odometryExternalListener);

  while (1) {

    printf("connecting\n");

    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
      printf("connection with server failed %d\n", errno);
      continue;
    }

    printf("TCP clien connection to "
           "%s, port %d...\n", serv_ip, BIND_PORT);

    while (true) {
      // send twist message
      {
        Twist message;
        message.mutable_linear()->set_x(1);
        message.mutable_linear()->set_y(2);
        message.mutable_linear()->set_z(3);
        message.mutable_angular()->set_x(4);
        message.mutable_angular()->set_y(5);
        message.mutable_angular()->set_z(6);

        string data;
        if (!message.SerializeToString(&data)) {
          cerr << "Failed to write cmd vel." << endl;
        } else {
          TF_ClearMsg(&msg);
          msg.type = SYNAPSE_IN_CMD_VEL_TOPIC;
          msg.len = data.length();
          msg.data = (const uint8_t *)data.c_str();
          TF_Send(tf0, &msg);
        }
      }

      // receive messages
      {
        uint8_t rx1_buf[RX_BUF_SIZE];
        int len = recv(sockfd, rx1_buf, sizeof(rx1_buf), 0);
        for (int i=0;i < len; i++) {
          TF_AcceptChar(tf0, rx1_buf[i]);
        }
      }

      // should move tf tick to a clock thread
      TF_Tick(tf0);
    }
  }
}
#endif

int main() {
#if defined(CONFIG_SYNAPTIC_GAP_ETHERNET)
  pthread_t ptid_ethernet;
  pthread_create(&ptid_ethernet, NULL, &ethernet_entry_point, NULL);
#endif

#if defined(CONFIG_SYNAPTIC_GAP_UART)
  pthread_t ptid_uart;
  pthread_create(&ptid_uart, NULL, &uart_entry_point, NULL);
#endif

#if defined(CONFIG_SYNAPTIC_GAP_ETHERNET)
  pthread_join(ptid_ethernet, NULL);
#endif

#if defined(CONFIG_SYNAPTIC_GAP_UART)
  pthread_join(ptid_uart, NULL);
#endif

  return 0;
};
