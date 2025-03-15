
#include <wsg_drive_data_driver/wsg_drive_data_driver.hpp>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <limits>
#include <fstream>
#include "pb_encode.h"
#include "pb_decode.h"
#include "wsg_cal_com.pb.h"
#include "wsg_drive_data.pb.h"

#define LOCALHOST_ADDRESS       "localhost:1883"
#define CLIENTID                "wsg_drive_data_node"
#define WSG_ESP_PI_TOPIC        "esp/wsg_dec_esp"
#define WSG_PI_ESP_TOPIC        "esp/wsg_dec_pi"
#define WSG_DRIVE_DATA_TOPIC      "esp/wsg_drive_data"
#define QOS                 2
#define TIMEOUT             10000L

namespace wsg_drive_data_driver {

WSGDriveDataDriver::WSGDriveDataDriver() : Node("wsg_drive_data_driver") {
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_ = create_publisher<baja_msgs::msg::Observation>("mqtt_data", rclcpp::QoS(1000).best_effort().durability_volatile(), pub_options);
    connect_to_broker();
    subscribe_to_topics();

    std::ofstream csvFile("src/bjr_packages/car/data.txt");
    csvFile.close();
}

WSGDriveDataDriver::~WSGDriveDataDriver() {
    unsubscribe_from_topics();
    disconnect_from_broker();
    MQTTClient_destroy(&client_);
}


void WSGDriveDataDriver::connect_to_broker() {
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

void WSGDriveDataDriver::subscribe_to_topics() {
    int rc;
    if ((rc = MQTTClient_subscribe(client_, WSG_DRIVE_DATA_TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
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

void WSGDriveDataDriver::unsubscribe_from_topics() {
    int rc;
    if ((rc = MQTTClient_unsubscribe(client_, WSG_DRIVE_DATA_TOPIC)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to unsubscribe, return code %d", rc);
    }

    if ((rc = MQTTClient_unsubscribe(client_, WSG_ESP_PI_TOPIC)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to unsubscribe, return code %d", rc);
    }
}

void WSGDriveDataDriver::disconnect_from_broker() {
    int rc;
    if ((rc = MQTTClient_disconnect(client_, 10000)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disconnect, return code %d", rc);
    }
}

int WSGDriveDataDriver::message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    WSGDriveDataDriver* driver = static_cast<WSGDriveDataDriver*>(context);
    
    MQTTClient_deliveryToken token;
    MQTTClient_message echo_message = MQTTClient_message_initializer;

    std::string topic_str(topicName);

    do {
        if (topic_str == WSG_ESP_PI_TOPIC) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Message from ESP32 Waiting");
            // Decode Polling Message
            {
                cal_command_t decoded_cmd = cal_command_t_init_zero;
                pb_istream_t istream = pb_istream_from_buffer((const pb_byte_t*) message->payload, message->payloadlen);

                if (!pb_decode(&istream, cal_command_t_fields, &decoded_cmd)) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error decoding ESP Command: %s", PB_GET_ERROR(&istream));
                    break;
                }

                if (decoded_cmd.command != 1) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Did not received polling signal");
                    break;
                }

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received polling signal");
            }
            
            // Send Start Message
            {
                cal_command_t cmd = cal_command_t_init_zero;
                cmd.command = 2;

                uint8_t buffer[128] = {0};
                pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

                if (!pb_encode(&ostream, cal_command_t_fields, &cmd)) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error encoding cal_command_t: %s", PB_GET_ERROR(&ostream));
                    break;
                }

                MQTTClient_deliveryToken token;
                MQTTClient_message start_message = MQTTClient_message_initializer;
                    
                start_message.payload = buffer;
                start_message.payloadlen = (int) ostream.bytes_written;
                start_message.qos = 0;
                start_message.retained = 0;
            
                MQTTClient_publishMessage(driver->client_, WSG_PI_ESP_TOPIC, &start_message, &token);

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "published drive start command");
            }
        } else if (topic_str == WSG_DRIVE_DATA_TOPIC) { 
            drive_data_t drive_data = drive_data_t_init_zero;
            pb_istream_t istream = pb_istream_from_buffer((const pb_byte_t*) message->payload, message->payloadlen);

            if (!pb_decode(&istream, drive_data_t_fields, &drive_data)) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error decoding ESP Data: %s", PB_GET_ERROR(&istream));
                break;
            }

            std::string mac_str(drive_data.mac_address);

            uint8_t channel_type;
            if (mac_str == "84:FC:E6:00:99:98") {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s identified as AXLE_TORQUE_REAR_LEFT", mac_str.data());
                channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_REAR_LEFT; //after swap
            } else if (mac_str == "8C:BF:EA:CB:AD:F4") {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s identified as AXLE_TORQUE_REAR_RIGHT\n", mac_str.data());
                channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_REAR_RIGHT;
            } else if (mac_str == "54:32:04:22:5A:40") {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s identified as AXLE_TORQUE_FRONT_LEFT");
                channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_LEFT; //after swap
            } else if (mac_str == "EC:DA:3B:BE:75:68") {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s identified as AXLE_TORQUE_FRONT_RIGHT", mac_str.data());
                channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_RIGHT;
            } else {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s identified as MISCELLANEOUS", mac_str.data());
                channel_type = baja_msgs::msg::AnalogChannel::MISCELLANEOUS;
            }

            std::ofstream csvFile("src/bjr_packages/car/data.txt", std::ios::app);

            // Reuse AnalogChannel object
            baja_msgs::msg::AnalogChannel analog_ch;
            analog_ch.channel_type = channel_type;

            // Needs to put time one to one
            baja_msgs::msg::Observation observation;
            for (int i = 0; i < 1000; i++) {
                observation.timestamp.ts = drive_data.base_time + drive_data.time_offset[i];
                analog_ch.encoded_analog_points = static_cast<double>(drive_data.voltage[i]);
                observation.analog_ch.push_back(analog_ch);

                if (csvFile.is_open()) {
                    csvFile << int(channel_type) << "," << drive_data.base_time + drive_data.time_offset[i] <<"," << drive_data.voltage[i] << ",\n";
                }
            }
            driver->publisher_->publish(observation);

            csvFile.close();  // Close the file after writing
        }
    } while (false);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    return 1;
}

void WSGDriveDataDriver::delivery_complete(void *context, MQTTClient_deliveryToken dt) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message with token value %d delivery confirmed", dt);
}

void WSGDriveDataDriver::connection_lost(void *context, char *cause) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection lost");
    if (cause)
    	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "     cause: %s", cause);
}


} // namespace wsg_drive_data_driver
