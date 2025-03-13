
#include <wsg_calibration_driver/wsg_calibration_driver.hpp>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <limits>
#include "pb_encode.h"
#include "pb_decode.h"
#include "wsg_cal_com.pb.h"
#include "wsg_cal_data.pb.h"

#define LOCALHOST_ADDRESS       "localhost:1883"
#define CLIENTID                "wsg_calibration_node"
#define WSG_ESP_PI_TOPIC        "esp/wsg_cal_esp"
#define WSG_PI_ESP_TOPIC        "esp/wsg_cal_pi"
#define WSG_CAL_DATA_TOPIC      "esp/wsg_cal_data"
#define QOS                 2
#define TIMEOUT             10000L
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
    static uint32_t label_counter = 0;
    static std::deque<float> sample_volt_avgs_;
    static std::deque<float> sample_volt_min_;
    static std::deque<float> sample_volt_max_;

    WSGCalibrationDriver* driver = static_cast<WSGCalibrationDriver*>(context);
    
    MQTTClient_deliveryToken token;
    MQTTClient_message echo_message = MQTTClient_message_initializer;

    std::string topic_str(topicName);

    do {
        if (topic_str == WSG_ESP_PI_TOPIC) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Message from ESP32 in Calibration");
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
                cmd.command = 1;

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

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "published start command");
            }
        } else if (topic_str == WSG_CAL_DATA_TOPIC) {   
            cal_data_t cal_data = cal_data_t_init_zero;
            pb_istream_t istream = pb_istream_from_buffer((const pb_byte_t*) message->payload, message->payloadlen);

            if (!pb_decode(&istream, cal_data_t_fields, &cal_data)) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error decoding ESP Data: %s", PB_GET_ERROR(&istream));
                break;
            }

            float sample_average = 0;
            float sample_min = std::numeric_limits<float>::max();
            float sample_max = std::numeric_limits<float>::min();

            for (int i = 0; i < 40; i++) {
                sample_average += cal_data.voltage[i] / 40;
                sample_min = (cal_data.voltage[i] < sample_min) ? cal_data.voltage[i] : sample_min;
                sample_max = (cal_data.voltage[i] > sample_max) ? cal_data.voltage[i] : sample_max;
            }
            
            if (sample_volt_avgs_.size() == 10) {
                sample_volt_avgs_.pop_back();
                sample_volt_min_.pop_back();
                sample_volt_max_.pop_back();
            }

            sample_volt_avgs_.push_front(sample_average);
            sample_volt_min_.push_front(sample_min);
            sample_volt_max_.push_front(sample_max);

            float sample_10_average = 0;
            float sample_10_min = std::numeric_limits<float>::max();
            float sample_10_max = std::numeric_limits<float>::min();

            for (int i = 0; i < sample_volt_avgs_.size(); i++) {
                sample_10_average += sample_volt_avgs_[i] / sample_volt_avgs_.size();
                sample_10_min = (sample_volt_min_[i] < sample_10_min) ? sample_volt_min_[i] : sample_10_min;
                sample_10_max = (sample_volt_max_[i] > sample_10_max) ? sample_volt_max_[i] : sample_10_max;
            }

            if (label_counter % 10 == 0) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sample Avg\tSample Min\tSample Max\tSamples-10 Avg\tSamples-10 Min\tSamples-10 Max\t");
            }
            label_counter++;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f\t%f\t%f\t%f\t%f\t%f\t", sample_average, sample_min, sample_max, sample_10_average, sample_10_min, sample_10_max);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message from unknown topic recieved");
        }
    } while (false);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    return 1;
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
