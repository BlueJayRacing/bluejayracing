/*******************************************************************************
 * Copyright (c) 2012, 2023 IBM Corp., Ian Craggs
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
#include <mqueue.h>
#include <vector>
#include <unistd.h>

#include "ipc_config.h"
#include "baja_live_comm.pb.h"
#include "MQTTClient.h"

#define ADDRESS     "localhost:1883"
#define CLIENTID    "ExampleClientSub"
#define TOPIC       "esp32/+"
#define PAYLOAD     "Hello World!"
#define QOS         2
#define TIMEOUT     10000L


std::string serializeDoubleToBinaryString(double value) {
    // Cast the address of 'value' to a pointer to 'unsigned char'
    const unsigned char* p = reinterpret_cast<const unsigned char*>(&value);
    // Construct a string from the binary data of 'value'
    return std::string(p, p + sizeof(double));
}


volatile MQTTClient_deliveryToken deliveredtoken;
mqd_t rx_queue;
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{   
    //printf("Message arrived\n");
    //printf("     topic: %s\n", topicName);
    uint32_t start_time = (*((uint8_t*) message->payload) << 24) + (*((uint8_t*) message->payload + 1) << 16) + (*((uint8_t*) message->payload + 2) << 8) + (*((uint8_t*) message->payload + 3)); 

    std::string msg;

    for (int i = 0; i < 40; i++) {
        Timestamp* timestamp = new Timestamp();
        timestamp->set_ts(start_time + i);
        uint16_t val = *(((uint8_t*) message->payload) + 4 + 2 * i) * 256 + *(((uint8_t*) message->payload) + 4 + 2 * i + 1);
        printf("%d ", start_time + i);
        printf("%d\n", val);
        std::string data = serializeDoubleToBinaryString((double)val);
        AnalogChannel* channel = new AnalogChannel();
        channel->set_encoded_analog_points(data);

        if (strcmp(topicName, "esp32/48:31:B7:3F:DA:90") == 0) {
            channel->set_channel_type(AnalogChannel::AXLE_TORQUE_FRONT_RIGHT); // TODO: Gotta get correct channel type
        } else if (strcmp(topicName, "esp32/84:FC:E6:00:92:DC") == 0) {
    	    channel->set_channel_type(AnalogChannel::AXLE_TORQUE_REAR_RIGHT);
        }

        Observation observation;
        observation.set_allocated_timestamp(timestamp);
        observation.set_allocated_analog_ch(channel);
        observation.SerializeToString(&msg);

        int err = BajaIPC::send_message(rx_queue, msg);
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    if (cause)
    	printf("     cause: %s\n", cause);
}

int main(int argc, char* argv[])
{
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    rx_queue = BajaIPC::open_queue(CarIPC::MQTT_CLIENT_TO_BROKER_QUEUE, false);

    if (rx_queue == -1) {
    	std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    	return EXIT_FAILURE;
    }

    int rc;

    if ((rc = MQTTClient_create(&client, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to create client, return code %d\n", rc);
        rc = EXIT_FAILURE;
        goto exit;
    }

    if ((rc = MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to set callbacks, return code %d\n", rc);
        rc = EXIT_FAILURE;
        goto destroy_exit;
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        rc = EXIT_FAILURE;
        goto destroy_exit;
    }

    printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n"
           "Press Q<Enter> to quit\n\n", TOPIC, CLIENTID, QOS);
    if ((rc = MQTTClient_subscribe(client, TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
    {
    	printf("Failed to subscribe, return code %d\n", rc);
    	rc = EXIT_FAILURE;
    }
    else
    {
    	int ch;
    	do
    	{
        	ch = getchar();
    	} while (ch!='Q' && ch != 'q');

        if ((rc = MQTTClient_unsubscribe(client, TOPIC)) != MQTTCLIENT_SUCCESS)
        {
        	printf("Failed to unsubscribe, return code %d\n", rc);
        	rc = EXIT_FAILURE;
        }
    }

    if ((rc = MQTTClient_disconnect(client, 10000)) != MQTTCLIENT_SUCCESS)
    {
    	printf("Failed to disconnect, return code %d\n", rc);
    	rc = EXIT_FAILURE;
    }
destroy_exit:
    MQTTClient_destroy(&client);
exit:
    return rc;
}
