#include "transmit_prioritizer_driver/transmit_prioritizer_driver.hpp"


namespace transmit_prioritizer {

TransmitPrioritizerDriver::TransmitPrioritizerDriver() : Node("transmit_prioritizer_driver") {
    observation_sub_ = create_subscription<baja_msgs::msg::Observation>(
        "transmit_data", 10, std::bind(&TransmitPrioritizerDriver::observation_callback, this, std::placeholders::_1));

    radio_pub_ = create_publisher<std_msgs::msg::String>("radio_data", 10);
}

void TransmitPrioritizerDriver::observation_callback(const baja_msgs::msg::Observation::SharedPtr msg) {
    Observation proto_observation;
    translate_observation(*msg, proto_observation);
    live_comm_.add_observations()->CopyFrom(proto_observation);

    std::string serialized_data;
    live_comm_.SerializeToString(&serialized_data);
    std_msgs::msg::String ros_serialized_data;
    ros_serialized_data.data = serialized_data;

    if (serialized_data.size() >= MAX_PAYLOAD_SIZE) {
        radio_pub_->publish(ros_serialized_data);
        live_comm_.clear_observations();
    }
}

void TransmitPrioritizerDriver::translate_observation(const baja_msgs::msg::Observation& ros_observation, Observation& proto_observation) {
    translate_timestamp(ros_observation.timestamp, *proto_observation.mutable_timestamp());

    if (!ros_observation.gps.empty()) {
        translate_gps(ros_observation.gps[0], *proto_observation.mutable_gps());
    }
    if (!ros_observation.localization.empty()) {
        translate_localization(ros_observation.localization[0], *proto_observation.mutable_localization());
    }
    if (!ros_observation.communication.empty()) {
        translate_communication(ros_observation.communication[0], *proto_observation.mutable_communication());
    }
    if (!ros_observation.analog_ch.empty()) {
        translate_analog_channel(ros_observation.analog_ch[0], *proto_observation.mutable_analog_ch());
    }
    if (!ros_observation.car_state.empty()) {
        translate_car_state(ros_observation.car_state[0], *proto_observation.mutable_car_state());
    }
    if (!ros_observation.rtk_correction.empty()) {
        translate_rtk_correction(ros_observation.rtk_correction[0], *proto_observation.mutable_rtk_correction());
    }
}

void TransmitPrioritizerDriver::translate_timestamp(const baja_msgs::msg::Timestamp& ros_timestamp, Timestamp& proto_timestamp) {
    proto_timestamp.set_ts(ros_timestamp.ts);
}

void TransmitPrioritizerDriver::translate_gps(const baja_msgs::msg::GPS& ros_gps, GPS& proto_gps) {
    proto_gps.set_latitude(ros_gps.latitude);
    proto_gps.set_longitude(ros_gps.longitude);
    proto_gps.set_altitude(ros_gps.altitude);
}

void TransmitPrioritizerDriver::translate_localization(const baja_msgs::msg::Localization& ros_localization, Localization& proto_localization) {
    proto_localization.set_x(ros_localization.x);
    proto_localization.set_y(ros_localization.y);
    proto_localization.set_z(ros_localization.z);
    proto_localization.set_roll(ros_localization.roll);
    proto_localization.set_pitch(ros_localization.pitch);
    proto_localization.set_yaw(ros_localization.yaw);

    if (ros_localization.vx) {
        proto_localization.set_vx(ros_localization.vx);
    }
    if (ros_localization.vy) {
        proto_localization.set_vy(ros_localization.vy);
    }
    if (ros_localization.vz) {
        proto_localization.set_vz(ros_localization.vz);
    }
    if (ros_localization.wr) {
        proto_localization.set_wr(ros_localization.wr);
    }
    if (ros_localization.wp) {
        proto_localization.set_wp(ros_localization.wp);
    }
    if (ros_localization.wy) {
        proto_localization.set_wy(ros_localization.wy);
    }
}

void TransmitPrioritizerDriver::translate_communication(const baja_msgs::msg::Communication& ros_communication, Communication& proto_communication) {
    if (ros_communication.instruction) {
        proto_communication.set_instruction(static_cast<Communication::DriverInstruction>(ros_communication.instruction));
    }
    if (ros_communication.driver_response) {
        proto_communication.set_driver_response(static_cast<Communication::DriverResponse>(ros_communication.driver_response));
    }
}

void TransmitPrioritizerDriver::translate_analog_channel(const baja_msgs::msg::AnalogChannel& ros_analog_channel, AnalogChannel& proto_analog_channel) {
    proto_analog_channel.set_channel_type(static_cast<AnalogChannel::ChannelType>(ros_analog_channel.channel_type));
    proto_analog_channel.set_encoded_analog_points(serializeDoubleToBinaryString(ros_analog_channel.encoded_analog_points));
}

void TransmitPrioritizerDriver::translate_car_state(const baja_msgs::msg::CarState& ros_car_state, CarState& proto_car_state) {
    if (ros_car_state.engine_status) {
        proto_car_state.set_engine_status(static_cast<CarState::EngineStatus>(ros_car_state.engine_status));
    }
    if (ros_car_state.rollover_status) {
        proto_car_state.set_rollover_status(ros_car_state.rollover_status);
    }
}

void TransmitPrioritizerDriver::translate_rtk_correction(const baja_msgs::msg::RTKCorrection& ros_rtk_correction, RTKCorrection& proto_rtk_correction) {
    const char* rtk_msg = reinterpret_cast<const char*>(ros_rtk_correction.rtk_correction.data());
    proto_rtk_correction.set_rtk_correction(std::string(rtk_msg));
}

std::string TransmitPrioritizerDriver::serializeDoubleToBinaryString(double value) {
    const unsigned char* p = reinterpret_cast<const unsigned char*>(&value);
    return std::string(p, p + sizeof(double));
}

} // namespace transmit_prioritizer