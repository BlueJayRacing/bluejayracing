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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MQTTClient.h"
#include <chrono>
#include <ctime>
#include <thread>
#include <ostream>
#include <iostream>

#define ADDRESS     "localhost:1883"
#define CLIENTID    "ClientPub"
#define TOPIC       "esp32/send"
#define PAYLOAD     "send"
#define QOS         2
#define TIMEOUT     10000L

int main(int argc, char* argv[])
{
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc;

    if ((rc = MQTTClient_create(&client, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
         printf("Failed to create client, return code %d\n", rc);
         exit(EXIT_FAILURE);
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    char message[5] = "send";

    pubmsg.payload = message;
    pubmsg.payloadlen = (int)strlen(PAYLOAD);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;

    std::cout << message << std::endl;
    std::cout << TOPIC << std::endl;

    std::cout << "publish message set" << std::endl;

    auto log_time = std::chrono::system_clock::now();
    auto current_time = std::chrono::system_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - log_time);

    while (true) {
    	rc = MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
        std::cout << "message sent" << std::endl;

	if (rc != MQTTCLIENT_SUCCESS) {
	    std::cout << "message failed" << std::endl;
	}

	rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
	printf("Message with delivery token %d delivered\n", token);

	while (time_diff.count() < 300) {
	    current_time = std::chrono::system_clock::now();
	    time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - log_time);
	    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}

        current_time = std::chrono::system_clock::now();
	log_time = std::chrono::system_clock::now();
	time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - log_time);
    }

    MQTTClient_destroy(&client);
    return rc;
}
