
#include <wsg_calibration_driver/wsg_calibration_driver.hpp>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <baja_proto/pb_encode.c>

#define LOCALHOST_ADDRESS       "localhost:1883"
#define CLIENTID                "wsg_calibration_node"
#define WSG_ESP_PI_TOPIC        "esp/wsg_cal_esp"
#define WSG_PI_ESP_TOPIC        "esp/wsg_cal_pi"
#define WSG_CAL_DATA_TOPIC      "esp/wsg_cal_data"
#define QOS         0
#define TIMEOUT     10000L
#define MQTT_MESSAGE_LENGTH 200

namespace wsg_calibration_driver {

WSGCalibrationDriver::WSGCalibrationDriver() : Node("wsg_calibration_driver") {
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_ = create_publisher<baja_msgs::msg::Observation>("mqtt_data", rclcpp::QoS(1000).best_effort().durability_volatile(), pub_options);
    connect_to_broker();
    subscribe_to_topics();
}

WSGCalibrationDriver::~WSGCalibrationDriver() {
    unsubscribe_from_topics();
    disconnect_from_broker();
    MQTTClient_destroy(&client_);
}


void WSGCalibrationDriver::connect_to_broker() {
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    if ((rc = MQTTClient_create(&client_, LOCALHOST_ADDRESS, CLIENTID,
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

void WSGCalibrationDriver::subscribe_to_topics() {
    int rc;
    if ((rc = MQTTClient_subscribe(client_, WSG_CAL_DATA_TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to subscribe to calibration data topic, return code %d", rc);
        rclcpp::shutdown();
    }

    if ((rc = MQTTClient_subscribe(client_, WSG_ESP_PI_TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to subscribe to (ESP - PI) topic, return code %d", rc);
        rclcpp::shutdown();
    }
}

void WSGCalibrationDriver::unsubscribe_from_topics() {
    int rc;
    if ((rc = MQTTClient_unsubscribe(client_, WSG_CAL_DATA_TOPIC)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to unsubscribe, return code %d", rc);
    }
}

void WSGCalibrationDriver::disconnect_from_broker() {
    int rc;
    if ((rc = MQTTClient_disconnect(client_, 10000)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disconnect, return code %d", rc);
    }
}

int WSGCalibrationDriver::message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    WSGCalibrationDriver* driver = static_cast<WSGCalibrationDriver*>(context);
    
    MQTTClient_deliveryToken token;
    MQTTClient_message echo_message = MQTTClient_message_initializer;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received MQTT message (topic: %s, topicLen: %d) (message: %s)", topicName, topicLen, (char*) message->payload);

    if (strcmp(WSG_ESP_PI_TOPIC, topicName) == 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Message from ESP32 in Calibration");
 
    } else if (strcmp(WSG_CAL_DATA_TOPIC, topicName) == 0) {
        
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message from unknown topic recieved");
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    return 0;
}

void WSGCalibrationDriver::delivery_complete(void *context, MQTTClient_deliveryToken dt) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message with token value %d delivery confirmed", dt);
}

void WSGCalibrationDriver::connection_lost(void *context, char *cause) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection lost");
    if (cause)
    	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "     cause: %s", cause);
}


} // namespace mqtt_calibration_driver
