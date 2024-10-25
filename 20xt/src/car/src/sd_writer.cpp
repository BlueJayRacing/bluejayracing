#include <fstream>
#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "baja_live_comm.pb.h"
#include "ipc_config.h"

using namespace std;

void write_line(ofstream& f, const vector<string>& v);

void log_gps(ofstream& f, Observation& observation);

void log_localization(ofstream& f, Observation& observation);

void log_communication(ofstream& f, Observation& observation);

void log_analog_ch(ofstream& f, Observation& observation);

void log_car_state(ofstream& f, Observation& observation);

void log_rtk_correction(ofstream& f, Observation& observation);

/* Infinite loop attempting to read data from IPC queue and then write to disk */
int main()
{
    cout << "starting sd writer..." << endl;

    // Open queue
    const mqd_t data_queue = BajaIPC::open_queue(CarIPC::BROKER_TO_SD_WRITER_QUEUE, true);
    if (data_queue == -1) {
        cout << "Failed to get data queue. Errno " << errno << endl;
        return EXIT_FAILURE;
    }

    int NUM_FIELDS = 6;
    vector<ofstream> f; // everything except timestamp, which is packed within the data for each csv
    vector<string> filenames = {"gps.csv",       "localization.csv", "communication.csv",
                                "analog_ch.csv", "car_state.csv",    "rtk_correction.csv"};

    vector<vector<string>> headers({
        vector<string>({"latitude", "longitude", "altitude", "timestamp"}), // gps
        vector<string>(
            {"x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz", "wr", "wp", "wy, timestamp"}), // localization
        vector<string>({"instruction", "driver_response", "timestamp"}),                             // communication
        vector<string>({"channel_type", "encoded_analog_points", "timestamp"}),                      // analog channel
        vector<string>({"engine_status", "rollover_status", "timestamp"}),                           // car state
        vector<string>({"rtk_correction", "timestamp"})                                              // rtk
    });

    for (int i = 0; i < NUM_FIELDS; i++) {
        ofstream stream;
        stream.open("logs/" + filenames[i]);
        write_line(stream, headers[i]);
        f.push_back(move(stream));
    }

    // Main loop
    while (true) {
        string msg = BajaIPC::get_message(data_queue); // Blocking
        if (msg == "") {
            continue;
        }

        Observation observation;
        observation.ParseFromString(msg);

        if (observation.has_gps()) {
            log_gps(f[0], observation);
        }
        if (observation.has_localization()) {
            log_localization(f[1], observation);
        }
        if (observation.has_communication()) {
            log_communication(f[2], observation);
        }
        if (observation.has_analog_ch()) {
            log_analog_ch(f[3], observation);
        }
        if (observation.has_car_state()) {
            log_car_state(f[4], observation);
        }
        if (observation.has_rtk_correction()) {
            log_rtk_correction(f[5], observation);
        }
    }

    for (int i = 0; i < NUM_FIELDS; i++) {
        f[i].close();
    }
    return EXIT_SUCCESS;
}

double deserializeDoubleFromBinaryStringCasting(const std::string& buffer)
{
    const auto* p = reinterpret_cast<const double*>(buffer.data());
    return *p;
}

void write_line(ofstream& f, const vector<string>& v)
{
    for (string e : v) {
        f << e << ",";
    }
    f << "\n";
}

void log_gps(ofstream& f, Observation& observation)
{
    GPS gps    = observation.gps();
    string lat = to_string(gps.latitude()), lon = to_string(gps.longitude()), alt = to_string(gps.altitude());
    string ts = to_string(observation.timestamp().ts());
    vector<string> line({lat, lon, alt, ts});
    write_line(f, line);
}

void log_localization(ofstream& f, Observation& observation)
{
    Localization loc = observation.localization();
    string ts        = to_string(observation.timestamp().ts());
    string x = to_string(loc.x()), y = to_string(loc.y()), z = to_string(loc.z());
    string roll = to_string(loc.roll()), pitch = to_string(loc.pitch()), yaw = to_string(loc.yaw());
    string vx = loc.has_vx() ? to_string(loc.vx()) : "";
    string vy = loc.has_vy() ? to_string(loc.vy()) : "";
    string vz = loc.has_vz() ? to_string(loc.vz()) : "";
    string wr = loc.has_wr() ? to_string(loc.wr()) : "";
    string wp = loc.has_wp() ? to_string(loc.wp()) : "";
    string wy = loc.has_wy() ? to_string(loc.wy()) : "";

    vector<string> line({x, y, z, roll, pitch, yaw, vx, vy, vz, wr, wp, wy, ts});
    write_line(f, line);
}

void log_communication(ofstream& f, Observation& observation)
{
    Communication com  = observation.communication();
    string ts          = to_string(observation.timestamp().ts());
    string instruction = "", driver_response = "";
    if (com.has_instruction()) {
        switch (com.instruction()) {
        case Communication::STOP_IMMEDIATELY:
            instruction = "STOP_IMMEDIATELY";
            break;
        case Communication::STOP_FOR_PIT:
            instruction = "STOP_FOR_PIT";
            break;
        default:
            instruction = "";
        }
    }
    if (com.has_driver_response()) {
        switch (com.driver_response()) {
        case Communication::SEND_HELP:
            driver_response = "SEND_HELP";
            break;
        case Communication::PIT_REQUEST:
            driver_response = "PIT_REQUEST";
            break;
        default:
            driver_response = "";
        }
    }
    vector<string> line({instruction, driver_response, ts});
    write_line(f, line);
}

void log_analog_ch(ofstream& f, Observation& observation)
{
    cout << observation.DebugString() << endl;
    AnalogChannel ana = observation.analog_ch();
    string ts         = to_string(observation.timestamp().ts());
    string channel_type;
    cout << ana.channel_type() << endl;
    switch (ana.channel_type()) {
    case AnalogChannel::AXLE_TORQUE_FRONT_LEFT:
        channel_type = "AXLE_TORQUE_FRONT_LEFT";
        break;
    case AnalogChannel::AXLE_TORQUE_FRONT_RIGHT:
        channel_type = "AXLE_TORQUE_FRONT_RIGHT";
        break;
    case AnalogChannel::AXLE_TORQUE_REAR_LEFT:
        channel_type = "AXLE_TORQUE_REAR_LEFT";
        break;
    case AnalogChannel::AXLE_TORQUE_REAR_RIGHT:
        channel_type = "AXLE_TORQUE_REAR_RIGHT";
        break;

    case AnalogChannel::AXLE_RPM_FRONT_RIGHT:
        channel_type = "AXLE_RPM_FRONT_RIGHT";
        break;
    case AnalogChannel::AXLE_RPM_FRONT_LEFT:
        channel_type = "AXLE_RPM_FRONT_LEFT";
        break;
    case AnalogChannel::AXLE_RPM_REAR:
        channel_type = "AXLE_RPM_REAR";
        break;

    case AnalogChannel::SHOCK_LEN_FRONT_RIGHT:
        channel_type = "SHOCK_LEN_FRONT_RIGHT";
        break;
    case AnalogChannel::SHOCK_LEN_FRONT_LEFT:
        channel_type = "SHOCK_LEN_FRONT_LEFT";
        break;
    case AnalogChannel::SHOCK_LEN_REAR_RIGHT:
        channel_type = "SHOCK_LEN_REAR_RIGHT";
        break;
    case AnalogChannel::SHOCK_LEN_REAR_LEFT:
        channel_type = "SHOCK_LEN_REAR_LEFT";
        break;

    case AnalogChannel::STEERING_DATA:
        channel_type = "STEERING_DATA";
        break;
    case AnalogChannel::TACHOMETER:
        channel_type = "TACHOMETER";
        break;
    case AnalogChannel::POWER_USE:
        channel_type = "POWER_USE";
        break;

    case AnalogChannel::BRAKE_PRESSURE_FRONT:
        channel_type = "BRAKE_PRESSURE_FRONT";
        break;
    case AnalogChannel::BRAKE_PRESSURE_REAR:
        channel_type = "BRAKE_PRESSURE_REAR";
        break;
    case AnalogChannel::MISCELLANEOUS:
        channel_type = "NONE";
        break;
    default:
        channel_type = "";
    }
    string encoded_analog_points = to_string(deserializeDoubleFromBinaryStringCasting(ana.encoded_analog_points()));
    vector<string> line({channel_type, encoded_analog_points, ts});
    write_line(f, line);
}

void log_car_state(ofstream& f, Observation& observation)
{
    CarState cs          = observation.car_state();
    string ts            = to_string(observation.timestamp().ts());
    string engine_status = "";
    if (cs.has_engine_status()) {
        switch (cs.engine_status()) {
        case CarState::ENGINE_NORMAL:
            engine_status = "ENGINE_NORMAL";
            break;
        case CarState::ENGINE_IDLING:
            engine_status = "ENGINE_IDLING";
            break;
        case CarState::ENGINE_CHOKED:
            engine_status = "ENGINE_CHOKED";
            break;
        case CarState::ENGINE_UNDERPERFORMING:
            engine_status = "ENGINE_UNDERPERFORMING";
            break;
        case CarState::ENGINE_OFF:
            engine_status = "ENGINE_OFF";
            break;
        }
    }
    string rollover_status = cs.has_rollover_status() ? to_string(cs.rollover_status()) : "";
    vector<string> line({engine_status, rollover_status, ts});
    write_line(f, line);
}

void log_rtk_correction(ofstream& f, Observation& observation)
{
    RTKCorrection rtk     = observation.rtk_correction();
    string ts             = to_string(observation.timestamp().ts());
    string rtk_correction = rtk.rtk_correction();
    vector<string> line({rtk_correction, ts});
    write_line(f, line);
}