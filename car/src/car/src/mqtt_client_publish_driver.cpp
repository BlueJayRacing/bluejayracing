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

#include <bitset>
#include <chrono>
#include <string>
#include <time.h>

#include "mqtt_client_publish_driver/mqtt_client_publish_driver.hpp"

#define ADDRESS  "localhost:1883"
#define CLIENTID "ClientPub"
#define TOPIC    "esp32/can_send"
#define PAYLOAD  "SEND"
#define QOS      2
#define TIMEOUT  10000L

using namespace std::chrono_literals;

namespace mqtt_client_publish_driver
{

std::string uint64_to_byte_string(uint64_t value)
{
    std::array<char, 8> bytes;
    for (int i = 0; i < 8; ++i) {
        bytes[i] = static_cast<char>(value & 0xFF);
        value >>= 8;
    }
    return std::string(bytes.begin(), bytes.end());
}

MQTTClientPublishDriver::MQTTClientPublishDriver() : Node("mqtt_client_publish_driver")
{
    timer = create_wall_timer(30s, std::bind(&MQTTClientPublishDriver::publish_message_callback, this));
    connect_to_broker();
    publish_message_callback();
}

MQTTClientPublishDriver::~MQTTClientPublishDriver()
{
    disconnect_from_broker();
    MQTTClient_destroy(&client_);
}

void MQTTClientPublishDriver::connect_to_broker()
{
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    if ((rc = MQTTClient_create(&client_, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL)) !=
        MQTTCLIENT_SUCCESS) {
        printf("Failed to create client, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession      = 1;
    if ((rc = MQTTClient_connect(client_, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
}

void MQTTClientPublishDriver::disconnect_from_broker()
{
    int rc;
    if ((rc = MQTTClient_disconnect(client_, 10000)) != MQTTCLIENT_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disconnect, return code %d", rc);
    }
}

void MQTTClientPublishDriver::publish_message_callback()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "publish_message_callback");
    MQTTClient_deliveryToken token;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;

    std::string mqtt_message = std::string("SEND");

    auto curr_time = std::chrono::high_resolution_clock::now();

    uint64_t microseconds = curr_time.time_since_epoch() / 1.0us;

    mqtt_message += uint64_to_byte_string(microseconds);

    pubmsg.payload = const_cast<char*>(mqtt_message.c_str());
    ;
    pubmsg.payloadlen = (int)mqtt_message.length();
    pubmsg.qos        = QOS;
    pubmsg.retained   = 0;

    // std::cout << mqtt_message << std::endl;
    // std::cout << TOPIC << std::endl;

    MQTTClient_publishMessage(client_, TOPIC, &pubmsg, &token);

    MQTTClient_waitForCompletion(client_, token, TIMEOUT);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "published mqtt start message with time %llu", microseconds);

    // std::cout << "Message sent" << std::endl;
}

} // namespace mqtt_client_publish_driver
