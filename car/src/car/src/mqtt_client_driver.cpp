// mqtt_client_driver.cpp
#include "mqtt_client_driver/mqtt_client_driver.hpp"
#include <cstring>
#include <unistd.h>
#include <chrono>

#define ADDRESS     "localhost:1883"
#define CLIENTID    "mqtt_data_client"
#define TOPIC       "esp32/data"
#define QOS         0
#define TIMEOUT     10000L
#define MQTT_MESSAGE_LENGTH 200

static const char* HEX_CHARS = "0123456789ABCDEF";
static const char* HEX_ARRAY = 
    "000102030405060708090A0B0C0D0E0F"
    "101112131415161718191A1B1C1D1E1F"
    "202122232425262728292A2B2C2D2E2F"
    "303132333435363738393A3B3C3D3E3F"
    "404142434445464748494A4B4C4D4E4F"
    "505152535455565758595A5B5C5D5E5F"
    "606162636465666768696A6B6C6D6E6F"
    "707172737475767778797A7B7C7D7E7F"
    "808182838485868788898A8B8C8D8E8F"
    "909192939495969798999A9B9C9D9E9F"
    "A0A1A2A3A4A5A6A7A8A9AAABACADAEAF"
    "B0B1B2B3B4B5B6B7B8B9BABBBCBDBEBF"
    "C0C1C2C3C4C5C6C7C8C9CACBCCCDCECF"
    "D0D1D2D3D4D5D6D7D8D9DADBDCDDDEDF"
    "E0E1E2E3E4E5E6E7E8E9EAEBECEDEEEF"
    "F0F1F2F3F4F5F6F7F8F9FAFBFCFDFEFF";

// Lookup table for hex char to int conversion
constexpr std::array<uint8_t, 256> HEX_VALUES = []() {
    std::array<uint8_t, 256> arr{};
    for (int i = 0; i < 256; i++) {
        if (i >= '0' && i <= '9') arr[i] = i - '0';
        else if (i >= 'A' && i <= 'F') arr[i] = i - 'A' + 10;
        else if (i >= 'a' && i <= 'f') arr[i] = i - 'a' + 10;
        else arr[i] = 0xFF;  // Invalid hex character
    }
    return arr;
}();


namespace mqtt_client_driver {

MQTTClientDriver::MQTTClientDriver() : Node("mqtt_client_driver") {
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_ = create_publisher<baja_msgs::msg::Observation>("mqtt_data", rclcpp::QoS(1000).best_effort().durability_volatile(), pub_options);
    connect_to_broker();
    subscribe_to_topics();
}

MQTTClientDriver::~MQTTClientDriver() {
    unsubscribe_from_topics();
    disconnect_from_broker();
    MQTTClient_destroy(&client_);
}

void MQTTClientDriver::connect_to_broker() {
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

void MQTTClientDriver::subscribe_to_topics() {
    int rc;
    if ((rc = MQTTClient_subscribe(client_, TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to subscribe, return code %d", rc);
        rclcpp::shutdown();
    }
}

void MQTTClientDriver::unsubscribe_from_topics() {
    int rc;
    if ((rc = MQTTClient_unsubscribe(client_, TOPIC)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to unsubscribe, return code %d", rc);
    }
}

void MQTTClientDriver::disconnect_from_broker() {
    int rc;
    if ((rc = MQTTClient_disconnect(client_, 10000)) != MQTTCLIENT_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disconnect, return code %d", rc);
    }
}

struct mqtt_data {
    uint32_t data_diff_micro;
    uint16_t data_point;
};

struct mqtt_message {
    uint64_t start_micro;
    uint8_t mac_address[6];
    mqtt_data data[MQTT_MESSAGE_LENGTH];
};


std::vector<uint8_t> hex_string_to_bytes(std::string_view hex) {
    std::vector<uint8_t> bytes;
    bytes.reserve(hex.length() / 2);

    for (size_t i = 0; i < hex.length(); i += 2) {
        uint8_t high = HEX_VALUES[static_cast<unsigned char>(hex[i])];
        uint8_t low = HEX_VALUES[static_cast<unsigned char>(hex[i + 1])];
        
        if (high == 0xFF || low == 0xFF) {
            // Handle invalid hex characters
            continue;  // or throw an exception, depending on your error handling strategy
        }
        
        bytes.push_back((high << 4) | low);
    }

    return bytes;
}

// Helper functions for deserialization (same as before)
uint16_t deserialize_uint16(const uint8_t* buffer) {
    return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
}

uint32_t deserialize_uint32(const uint8_t* buffer) {
    return static_cast<uint32_t>(buffer[0]) |
           (static_cast<uint32_t>(buffer[1]) << 8) |
           (static_cast<uint32_t>(buffer[2]) << 16) |
           (static_cast<uint32_t>(buffer[3]) << 24);
}

uint64_t deserialize_uint64(const uint8_t* buffer) {
    return static_cast<uint64_t>(buffer[0]) |
           (static_cast<uint64_t>(buffer[1]) << 8) |
           (static_cast<uint64_t>(buffer[2]) << 16) |
           (static_cast<uint64_t>(buffer[3]) << 24) |
           (static_cast<uint64_t>(buffer[4]) << 32) |
           (static_cast<uint64_t>(buffer[5]) << 40) |
           (static_cast<uint64_t>(buffer[6]) << 48) |
           (static_cast<uint64_t>(buffer[7]) << 56);
}

mqtt_message deserialize_mqtt_message(const std::vector<uint8_t>& buffer) {
    mqtt_message msg;
    size_t offset = 0;

    // Deserialize start_micro
    msg.start_micro = deserialize_uint64(buffer.data() + offset);
    offset += sizeof(uint64_t);

    // Deserialize mac_address
    std::memcpy(msg.mac_address, buffer.data() + offset, 6);
    offset += 6;

    // Deserialize data array
    for (int i = 0; i < MQTT_MESSAGE_LENGTH && offset < buffer.size(); ++i) {
        msg.data[i].data_diff_micro = deserialize_uint32(buffer.data() + offset);
        offset += sizeof(uint32_t);

        msg.data[i].data_point = deserialize_uint16(buffer.data() + offset);
        offset += sizeof(uint16_t);
    }

    return msg;
}

void print_bytes(const unsigned char* data, std::size_t size) {
    for (std::size_t i = 0; i < size; i++) {
        std::cout << std::uppercase << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(data[i]) << ' ';
    }
    std::cout << std::dec << std::endl;
}


int MQTTClientDriver::message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    // auto start_total = std::chrono::high_resolution_clock::now();

    MQTTClientDriver* driver = static_cast<MQTTClientDriver*>(context);
    
    // auto start_initial_processing = std::chrono::high_resolution_clock::now();

    // Convert the payload to a hex string
    std::string hex_payload;
    if (message->payloadlen * 2 > 2428) {
        RCLCPP_ERROR(driver->get_logger(), "payload is too long of length %i", message->payloadlen);
    }
    hex_payload.reserve(std::max(message->payloadlen * 2, 4000));
    const unsigned char* payload = static_cast<const unsigned char*>(message->payload);
    for (int i = 0; i < message->payloadlen; ++i) {
        unsigned char c = payload[i];
        hex_payload.append(&HEX_ARRAY[c * 2], 2);
    }

    // Convert hex string to byte vector
    std::vector<uint8_t> byte_data = hex_string_to_bytes(hex_payload);

    mqtt_message mqtt_msg = deserialize_mqtt_message(byte_data);
    
    uint64_t start_time = mqtt_msg.start_micro;
    
    // Optimized MAC address conversion
    std::string mac_str;
    mac_str.reserve(17);  // 6 bytes * 2 chars + 5 colons
    for (int i = 0; i < 6; ++i) {
        if (i > 0) mac_str += ':';
        mac_str += HEX_CHARS[(mqtt_msg.mac_address[i] & 0xF0) >> 4];
        mac_str += HEX_CHARS[mqtt_msg.mac_address[i] & 0x0F];
    }
    
    // auto end_initial_processing = std::chrono::high_resolution_clock::now();
    
     // Determine the channel type based on the MAC address
    uint8_t channel_type;
    if (mac_str == "84:FC:E6:00:99:98") {
        channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_REAR_LEFT; //after swap
    } else if (mac_str == "EC:DA:3B:BE:81:7C") {
        channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_REAR_RIGHT;
    } else if (mac_str == "54:32:04:22:5A:40") {
        channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_LEFT; //after swap
    } else if (mac_str == "EC:DA:3B:BE:75:68") {
        channel_type = baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_RIGHT;
    } else {
        RCLCPP_ERROR(driver->get_logger(), "Unknown MAC address: %s, payload of len %i", mac_str.c_str(), message->payloadlen);
        channel_type = baja_msgs::msg::AnalogChannel::MISCELLANEOUS;
    }

    // Reuse AnalogChannel object
    baja_msgs::msg::AnalogChannel analog_ch;
    analog_ch.channel_type = channel_type;

    // auto start_publishing_loop = std::chrono::high_resolution_clock::now();
    baja_msgs::msg::Observation observation;
    for (int i = 0; i < MQTT_MESSAGE_LENGTH; i++) {
        if (mqtt_msg.data[i].data_diff_micro == 0 && mqtt_msg.data[i].data_point == 0) break;
        
        
        observation.timestamp.ts = start_time + mqtt_msg.data[i].data_diff_micro;
        analog_ch.encoded_analog_points = static_cast<double>(mqtt_msg.data[i].data_point);
        observation.analog_ch.push_back(analog_ch);

        
    }
    driver->publisher_->publish(observation);
    // auto end_publishing_loop = std::chrono::high_resolution_clock::now();

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    // auto end_total = std::chrono::high_resolution_clock::now();

    // Calculate durations
    // auto initial_processing_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_initial_processing - start_initial_processing).count();
    // auto publishing_loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_publishing_loop - start_publishing_loop).count();
    // auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total).count();

    // Log the benchmarking results
    // RCLCPP_INFO(driver->get_logger(), "Benchmarking results (microseconds):");
    // RCLCPP_INFO(driver->get_logger(), "  Initial processing: %ld", initial_processing_duration);
    // RCLCPP_INFO(driver->get_logger(), "  Publishing loop: %ld", publishing_loop_duration);
    // RCLCPP_INFO(driver->get_logger(), "  Total processing time: %ld", total_duration);

    return 0;
}

void MQTTClientDriver::delivery_complete(void *context, MQTTClient_deliveryToken dt) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message with token value %d delivery confirmed", dt);
}

void MQTTClientDriver::connection_lost(void *context, char *cause) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection lost");
    if (cause)
    	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "     cause: %s", cause);
}


} // namespace mqtt_client