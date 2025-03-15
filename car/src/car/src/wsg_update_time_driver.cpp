/*******************************************************************************
 * Copyright (c) 2012, 2022 IBM Corp., Ian Craggs
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * and Eclipse Distribution License v1.0 which accompany this distribution. 
 *
 * The Eclipse Public License is available at 
 *   https://www.eclipse.org/legal/epl-2.0/
 * and the Eclipse Distribution License is available at 
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial contribution
 *******************************************************************************/

#include <chrono>
#include <string>
#include <time.h>
#include <bitset>

#include <wsg_update_time_driver/wsg_update_time_driver.hpp>
#include "pb_encode.h"
#include "pb_decode.h"
#include "wsg_pi_time.pb.h"

#define LOCALHOST_ADDRESS       "localhost:1883"
#define CLIENTID                "wsg_update_time_node"
#define WSG_PI_TIME_TOPIC       "esp/pi_time"
#define QOS         2
#define TIMEOUT     10000L

using namespace std::chrono_literals;

namespace wsg_update_time_driver {

std::string uint64_to_byte_string(uint64_t value) {
    std::array<char, 8> bytes;
    for (int i = 0; i < 8; ++i) {
        bytes[i] = static_cast<char>(value & 0xFF);
        value >>= 8;
    }
    return std::string(bytes.begin(), bytes.end());
}

WSGUpdateTimeDriver::WSGUpdateTimeDriver() : Node("mqtt_client_publish_driver") {
    timer = create_wall_timer(5s, std::bind(&WSGUpdateTimeDriver::publish_message_callback, this));
    connect_to_broker();
    publish_message_callback();
}

WSGUpdateTimeDriver::~WSGUpdateTimeDriver() {
    disconnect_from_broker();
    MQTTClient_destroy(&client_);
}

void WSGUpdateTimeDriver::connect_to_broker() {
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    if ((rc = MQTTClient_create(&client_, LOCALHOST_ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
         printf("Failed to create client, return code %d\n", rc);
         exit(EXIT_FAILURE);
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    if ((rc = MQTTClient_connect(client_, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
}

void WSGUpdateTimeDriver::disconnect_from_broker() {
    int rc;
    if ((rc = MQTTClient_disconnect(client_, 10000)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disconnect, return code %d", rc);
    }
}

void WSGUpdateTimeDriver::publish_message_callback() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "publish_message_callback");
    MQTTClient_deliveryToken token;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    
    auto curr_time = std::chrono::high_resolution_clock::now();    

    pi_time_t pi_time = pi_time_t_init_zero;
    pi_time.time_us = curr_time.time_since_epoch()/1.0us;

    uint8_t buffer[30] = {0};
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    if (!pb_encode(&ostream, pi_time_t_fields, &pi_time)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error encoding pi_time_t: %s", PB_GET_ERROR(&ostream));
        return;
    }

    pubmsg.payload = buffer;
    pubmsg.payloadlen = (int) ostream.bytes_written;
    pubmsg.qos = QOS;
    pubmsg.retained = 0;

    MQTTClient_publishMessage(client_, WSG_PI_TIME_TOPIC, &pubmsg, &token);

    MQTTClient_waitForCompletion(client_, token, TIMEOUT);
}


}

