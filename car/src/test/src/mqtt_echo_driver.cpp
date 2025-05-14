// mqtt_client_driver.cpp
#include "mqtt_echo_driver/mqtt_echo_driver.hpp"
#include <cstring>
#include <unistd.h>
#include <chrono>

#define ADDRESS     "localhost:1883"
#define CLIENTID    "mqtt_echo"
#define ECHO_TOPIC_PUBLISH "esp32/test_publish"       // The Topic that the client should publish to
#define ECHO_TOPIC_SUBSCRIBE  "esp32/test_subscribe"  // The Topic that the client should subscribe to in order to receive echo message
#define QOS         0
#define TIMEOUT     10000L
#define MQTT_MESSAGE_LENGTH 200

namespace mqtt_echo_driver {

MQTTEchoDriver::MQTTEchoDriver() : Node("mqtt_echo_driver") {
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_ = create_publisher<baja_msgs::msg::Observation>("mqtt_data", rclcpp::QoS(1000).best_effort().durability_volatile(), pub_options);
    connect_to_broker();
    subscribe_to_topics();
}

MQTTEchoDriver::~MQTTEchoDriver() {
    unsubscribe_from_topics();
    disconnect_from_broker();
    MQTTClient_destroy(&client_);
}

void MQTTEchoDriver::connect_to_broker() {
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    if ((rc = MQTTClient_create(&client_, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create client, return code %d", rc);
        rclcpp::shutdown();
        return;
    }

    if ((rc = MQTTClient_setCallbacks(client_, this, connection_lost, message_arrived, delivery_complete)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set callbacks, return code %d", rc);
        rclcpp::shutdown();
        return;
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    if ((rc = MQTTClient_connect(client_, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to connect, return code %d", rc);
        rclcpp::shutdown();
        return;
    }
}

void MQTTEchoDriver::subscribe_to_topics() {
    int rc;
    if ((rc = MQTTClient_subscribe(client_, ECHO_TOPIC_PUBLISH, QOS)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to subscribe, return code %d", rc);
        rclcpp::shutdown();
    }
}

void MQTTEchoDriver::unsubscribe_from_topics() {
    int rc;
    if ((rc = MQTTClient_unsubscribe(client_, ECHO_TOPIC_PUBLISH)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to unsubscribe, return code %d", rc);
    }
}

void MQTTEchoDriver::disconnect_from_broker() {
    int rc;
    if ((rc = MQTTClient_disconnect(client_, 10000)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disconnect, return code %d", rc);
    }
}

int MQTTEchoDriver::message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    MQTTEchoDriver* driver = static_cast<MQTTEchoDriver*>(context);
    
    MQTTClient_deliveryToken token;
    MQTTClient_message echo_message = MQTTClient_message_initializer;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received MQTT message (topic: %s, topicLen: %d) (message: %s)", topicName, topicLen, (char*) message->payload);

    echo_message.payload = message->payload;
    echo_message.payloadlen = (int) message->payloadlen;
    echo_message.qos = QOS;
    echo_message.retained = 0;

    MQTTClient_publishMessage(driver->client_, ECHO_TOPIC_SUBSCRIBE, &echo_message, &token);

    MQTTClient_waitForCompletion(driver->client_, token, TIMEOUT);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "published MQTT echo message");

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    return 1;
}

void MQTTEchoDriver::delivery_complete(void *context, MQTTClient_deliveryToken dt) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message with token value %d delivery confirmed", dt);
}

void MQTTEchoDriver::connection_lost(void *context, char *cause) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection lost");
    if (cause)
    	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "     cause: %s", cause);
}


} // namespace mqtt_echo_driver