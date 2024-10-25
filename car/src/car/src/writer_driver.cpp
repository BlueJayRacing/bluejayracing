// writer_driver.cpp
#include "writer_driver/writer_driver.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace writer_driver
{

WriterDriver::WriterDriver() : Node("writer_driver")
{
    std::vector<std::string> filenames = {"gps.csv",       "localization.csv", "communication.csv",
                                          "analog_ch.csv", "car_state.csv",    "rtk_correction.csv"};

    std::vector<std::vector<std::string>> headers({
        {"latitude", "longitude", "altitude", "timestamp"},                                     // gps
        {"x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz", "wr", "wp", "wy, timestamp"}, // localization
        {"instruction", "driver_response", "timestamp"},                                        // communication
        {"channel_type", "encoded_analog_points", "timestamp"},                                 // analog channel
        {"engine_status", "rollover_status", "timestamp"},                                      // car state
        {"rtk_correction", "timestamp"}                                                         // rtk
    });

    for (size_t i = 0; i < filenames.size(); i++) {
        std::ofstream stream;
        stream.open("/20xt_ws/src/rsp_baja/logs/" + filenames[i]);
        write_line(stream, headers[i]);
        f_.push_back(std::move(stream));
    }

    subscription_ = create_subscription<baja_msgs::msg::Observation>(
        "sd_writer_data", 10, std::bind(&WriterDriver::observation_callback, this, std::placeholders::_1));

    // subscription_did_ = create_subscription<inertial_sense_ros_humble_msgs::msg::DIDINS4>(
    //     "DID_INS_4", 100, std::bind(&WriterDriver::observation_callback, this, std::placeholders::_1));
}

void WriterDriver::observation_callback(const baja_msgs::msg::Observation::SharedPtr msg)
{
    if (msg->gps.size() > 0) {
        log_gps(*msg);
    }
    if (msg->localization.size() > 0) {
        log_localization(*msg);
    }
    if (msg->communication.size() > 0) {
        log_communication(*msg);
    }
    if (msg->analog_ch.size() > 0) {
        log_analog_ch(*msg);
    }
    if (msg->car_state.size() > 0) {
        log_car_state(*msg);
    }
    if (msg->rtk_correction.size() > 0) {
        log_rtk_correction(*msg);
    }
}

void WriterDriver::log_gps(const baja_msgs::msg::Observation& observation)
{
    auto gps        = observation.gps[0];
    std::string lat = std::to_string(gps.latitude);
    std::string lon = std::to_string(gps.longitude);
    std::string alt = std::to_string(gps.altitude);
    std::string ts  = std::to_string(observation.timestamp.ts);
    std::vector<std::string> line({lat, lon, alt, ts});
    write_line(f_[0], line);
}

void WriterDriver::log_localization(const baja_msgs::msg::Observation& observation)
{
    auto loc          = observation.localization[0];
    std::string ts    = std::to_string(observation.timestamp.ts);
    std::string x     = std::to_string(loc.x);
    std::string y     = std::to_string(loc.y);
    std::string z     = std::to_string(loc.z);
    std::string roll  = std::to_string(loc.roll);
    std::string pitch = std::to_string(loc.pitch);
    std::string yaw   = std::to_string(loc.yaw);
    std::string vx    = loc.vx ? std::to_string(loc.vx) : "";
    std::string vy    = loc.vy ? std::to_string(loc.vy) : "";
    std::string vz    = loc.vz ? std::to_string(loc.vz) : "";
    std::string wr    = loc.wr ? std::to_string(loc.wr) : "";
    std::string wp    = loc.wp ? std::to_string(loc.wp) : "";
    std::string wy    = loc.wy ? std::to_string(loc.wy) : "";

    std::vector<std::string> line({x, y, z, roll, pitch, yaw, vx, vy, vz, wr, wp, wy, ts});
    write_line(f_[1], line);
}

void WriterDriver::log_communication(const baja_msgs::msg::Observation& observation)
{
    auto com                    = observation.communication[0];
    std::string ts              = std::to_string(observation.timestamp.ts);
    std::string instruction     = "";
    std::string driver_response = "";
    if (com.instruction != -1) {
        switch (com.instruction) {
        case baja_msgs::msg::Communication::STOP_IMMEDIATELY:
            instruction = "STOP_IMMEDIATELY";
            break;
        case baja_msgs::msg::Communication::STOP_FOR_PIT:
            instruction = "STOP_FOR_PIT";
            break;
        default:
            instruction = "";
        }
    }
    if (com.driver_response != -1) {
        switch (com.driver_response) {
        case baja_msgs::msg::Communication::SEND_HELP:
            driver_response = "SEND_HELP";
            break;
        case baja_msgs::msg::Communication::PIT_REQUEST:
            driver_response = "PIT_REQUEST";
            break;
        default:
            driver_response = "";
        }
    }
    std::vector<std::string> line({instruction, driver_response, ts});
    write_line(f_[2], line);
}

void WriterDriver::log_analog_ch(const baja_msgs::msg::Observation& observation)
{
    auto ana       = observation.analog_ch[0];
    std::string ts = std::to_string(observation.timestamp.ts);
    std::string channel_type;
    switch (ana.channel_type) {
    case baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_LEFT:
        channel_type = "AXLE_TORQUE_FRONT_LEFT";
        break;
    case baja_msgs::msg::AnalogChannel::AXLE_TORQUE_FRONT_RIGHT:
        channel_type = "AXLE_TORQUE_FRONT_RIGHT";
        break;
    case baja_msgs::msg::AnalogChannel::AXLE_TORQUE_REAR_LEFT:
        channel_type = "AXLE_TORQUE_REAR_LEFT";
        break;
    case baja_msgs::msg::AnalogChannel::AXLE_TORQUE_REAR_RIGHT:
        channel_type = "AXLE_TORQUE_REAR_RIGHT";
        break;
    case baja_msgs::msg::AnalogChannel::AXLE_RPM_FRONT_RIGHT:
        channel_type = "AXLE_RPM_FRONT_RIGHT";
        break;
    case baja_msgs::msg::AnalogChannel::AXLE_RPM_FRONT_LEFT:
        channel_type = "AXLE_RPM_FRONT_LEFT";
        break;
    case baja_msgs::msg::AnalogChannel::AXLE_RPM_REAR:
        channel_type = "AXLE_RPM_REAR";
        break;
    case baja_msgs::msg::AnalogChannel::SHOCK_LEN_FRONT_RIGHT:
        channel_type = "SHOCK_LEN_FRONT_RIGHT";
        break;
    case baja_msgs::msg::AnalogChannel::SHOCK_LEN_FRONT_LEFT:
        channel_type = "SHOCK_LEN_FRONT_LEFT";
        break;
    case baja_msgs::msg::AnalogChannel::SHOCK_LEN_REAR_RIGHT:
        channel_type = "SHOCK_LEN_REAR_RIGHT";
        break;
    case baja_msgs::msg::AnalogChannel::SHOCK_LEN_REAR_LEFT:
        channel_type = "SHOCK_LEN_REAR_LEFT";
        break;
    case baja_msgs::msg::AnalogChannel::STEERING_DATA:
        channel_type = "STEERING_DATA";
        break;
    case baja_msgs::msg::AnalogChannel::TACHOMETER:
        channel_type = "TACHOMETER";
        break;
    case baja_msgs::msg::AnalogChannel::POWER_USE:
        channel_type = "POWER_USE";
        break;
    case baja_msgs::msg::AnalogChannel::BRAKE_PRESSURE_FRONT:
        channel_type = "BRAKE_PRESSURE_FRONT";
        break;
    case baja_msgs::msg::AnalogChannel::BRAKE_PRESSURE_REAR:
        channel_type = "BRAKE_PRESSURE_REAR";
        break;
    case baja_msgs::msg::AnalogChannel::MISCELLANEOUS:
        channel_type = "NONE";
        break;
    default:
        channel_type = "";
    }
    std::string encoded_analog_points = std::to_string(ana.encoded_analog_points);
    std::vector<std::string> line({channel_type, encoded_analog_points, ts});
    write_line(f_[3], line);
}

void WriterDriver::log_car_state(const baja_msgs::msg::Observation& observation)
{
    auto cs                   = observation.car_state[0];
    std::string ts            = std::to_string(observation.timestamp.ts);
    std::string engine_status = "";
    if (cs.engine_status != -1) {
        switch (cs.engine_status) {
        case baja_msgs::msg::CarState::ENGINE_NORMAL:
            engine_status = "ENGINE_NORMAL";
            break;
        case baja_msgs::msg::CarState::ENGINE_IDLING:
            engine_status = "ENGINE_IDLING";
            break;
        case baja_msgs::msg::CarState::ENGINE_CHOKED:
            engine_status = "ENGINE_CHOKED";
            break;
        case baja_msgs::msg::CarState::ENGINE_UNDERPERFORMING:
            engine_status = "ENGINE_UNDERPERFORMING";
            break;
        case baja_msgs::msg::CarState::ENGINE_OFF:
            engine_status = "ENGINE_OFF";
            break;
        default:
            engine_status = "";
        }
    }
    std::string rollover_status = cs.rollover_status ? std::to_string(cs.rollover_status) : "";
    std::vector<std::string> line({engine_status, rollover_status, ts});
    write_line(f_[4], line);
}

void WriterDriver::log_rtk_correction(const baja_msgs::msg::Observation& observation)
{
    auto rtk                   = std::vector<char>(observation.rtk_correction[0].rtk_correction.begin(),
                                 observation.rtk_correction[0].rtk_correction.end());
    std::string ts             = std::to_string(observation.timestamp.ts);
    std::string rtk_correction = std::string(rtk.begin(), rtk.end());
    std::vector<std::string> line({rtk_correction, ts});
    write_line(f_[5], line);
}

void WriterDriver::write_line(std::ofstream& f, const std::vector<std::string>& v)
{
    for (const std::string& e : v) {
        f << e << ",";
    }
    f << "\n";
}

double WriterDriver::deserializeDoubleFromBinaryStringCasting(const std::string& buffer)
{
    const auto* p = reinterpret_cast<const double*>(buffer.data());
    return *p;
}

} // namespace writer_driver