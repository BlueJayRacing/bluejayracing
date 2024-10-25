#pragma once

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <string>
#include <yaml-cpp/yaml.h>

#include <InertialSense.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <data_sets.h>

#include "inertial_sense_ros_humble_msgs/msg/didins1.hpp"
#include "inertial_sense_ros_humble_msgs/msg/didins2.hpp"
#include "inertial_sense_ros_humble_msgs/msg/didins4.hpp"
#include "inertial_sense_ros_humble_msgs/msg/glonass_ephemeris.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gnss_ephemeris.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gnss_obs_vec.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gnss_observation.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gps.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gps_info.hpp"
#include "inertial_sense_ros_humble_msgs/msg/inl2_states.hpp"
#include "inertial_sense_ros_humble_msgs/msg/pre_int_imu.hpp"
#include "inertial_sense_ros_humble_msgs/msg/rtk_info.hpp"
#include "inertial_sense_ros_humble_msgs/msg/rtk_rel.hpp"

#include "inertial_sense_ros_humble_msgs/srv/firmwareupdate.hpp"
#include "inertial_sense_ros_humble_msgs/srv/refllaupdate.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ISConstants.h"
// #include "geometry/xform.h"

#define GPS_UNIX_OFFSET                                                                                                \
    315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in
              // seconds
#define LEAP_SECONDS                                                                                                   \
    18 // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless
       // there is some crazy earthquake or nuclear blast)
#define UNIX_TO_GPS_OFFSET     (GPS_UNIX_OFFSET - LEAP_SECONDS)
#define FIRMWARE_VERSION_CHAR0 1
#define FIRMWARE_VERSION_CHAR1 9
#define FIRMWARE_VERSION_CHAR2 0

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple)                                                          \
    IS_.BroadcastBinaryData(DID, __periodmultiple, [this](InertialSense* i, p_data_t* data, int pHandle) {             \
        /* ROS_INFO("Got message %d", DID);*/                                                                          \
        this->__cb_fun(DID, reinterpret_cast<__type*>(data->buf));                                                     \
    })

class InertialSenseROS : public rclcpp::Node {
  public:
    InertialSenseROS(bool configFlashParameters);
    void callback(p_data_t* data);
    void update();

    void load_params_srv();
    void load_params_yaml(YAML::Node node);
    template <typename Type> bool get_node_param_yaml(YAML::Node node, const std::string key, Type& val);
    template <typename Derived1>
    bool get_node_vector_yaml(YAML::Node node, const std::string key, int size, Derived1& val);
    void connect();
    bool firmware_compatibility_check();
    void set_navigation_dt_ms();
    void configure_flash_parameters();
    void configure_rtk();
    void connect_rtk_client(const std::string& RTK_correction_protocol, const std::string& RTK_server_IP,
                            const int RTK_server_port);
    void start_rtk_server(const std::string& RTK_server_IP, const int RTK_server_port);

    void configure_data_streams(bool startup);
    void configure_ascii_output();
    void start_log();

    template <typename T> void get_vector_flash_config(std::string param_name, uint32_t size, T& data);
    void get_flash_config();
    void reset_device();
    void flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t* const msg);
    bool flashConfigStreaming_ = false;

    // Serial Port Configuration
    std::string port_ = "/dev/ttyACM0";
    int baudrate_     = 921600;
    bool initialized_;
    bool log_enabled_        = false;
    bool covariance_enabled_ = false;

    std::string frame_id_ = "body";

    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    bool publishTf_ = true;
    tf2::Transform transform_NED;
    tf2::Transform transform_ENU;
    tf2::Transform transform_ECEF;
    enum
    {
        NED,
        ENU
    } ltcf;

    rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::DIDINS1>::SharedPtr did_ins_1_pub_;
    rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::DIDINS2>::SharedPtr did_ins_2_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ins_ned_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ins_ecef_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ins_enu_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr strobe_pub_;
    bool strobe_pub_created;
    rclcpp::TimerBase::SharedPtr obs_bundle_timer_;
    rclcpp::Time last_obs_time_1_;
    rclcpp::Time last_obs_time_2_;
    rclcpp::Time last_obs_time_base_;
    rclcpp::TimerBase::SharedPtr data_stream_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    inertial_sense_ros_humble_msgs::msg::GNSSObsVec gps1_obs_Vec_;
    inertial_sense_ros_humble_msgs::msg::GNSSObsVec gps2_obs_Vec_;
    inertial_sense_ros_humble_msgs::msg::GNSSObsVec base_obs_Vec_;

    bool rtk_connecting_                             = false;
    int RTK_connection_attempt_limit_                = 1;
    int RTK_connection_attempt_backoff_              = 2;
    int rtk_traffic_total_byte_count_                = 0;
    int rtk_data_transmission_interruption_count_    = 0;
    bool rtk_connectivity_watchdog_enabled_          = true;
    float rtk_connectivity_watchdog_timer_frequency_ = 1;
    int rtk_data_transmission_interruption_limit_    = 5;
    std::string RTK_server_mount_                    = "";
    std::string RTK_server_username_                 = "";
    std::string RTK_server_password_                 = "";
    std::string RTK_correction_protocol_             = "RTCM3";
    std::string RTK_server_IP_                       = "127.0.0.1";
    int RTK_server_port_                             = 7777;
    bool RTK_rover_                                  = false;
    bool RTK_rover_radio_enable_                     = false;
    bool RTK_base_USB_                               = false;
    bool RTK_base_serial_                            = false;
    bool RTK_base_TCP_                               = false;
    bool GNSS_Compass_                               = false;

    std::string gps1_type_  = "F9P";
    std::string gps1_topic_ = "gps1";
    std::string gps2_type_  = "F9P";
    std::string gps2_topic_ = "gps2";

    rclcpp::TimerBase::SharedPtr rtk_connectivity_watchdog_timer_;
    void start_rtk_connectivity_watchdog_timer();
    void stop_rtk_connectivity_watchdog_timer();
    void rtk_connectivity_watchdog_timer_callback();

    void INS1_callback(eDataIDs DID, const ins_1_t* const msg);
    void INS2_callback(eDataIDs DID, const ins_2_t* const msg);
    void INS4_callback(eDataIDs DID, const ins_4_t* const msg);
    void INL2_states_callback(eDataIDs DID, const inl2_states_t* const msg);
    void INS_covariance_callback(eDataIDs DID, const ros_covariance_pose_twist_t* const msg);
    void odom_ins_ned_callback(eDataIDs DID, const ins_2_t* const msg);
    void odom_ins_ecef_callback(eDataIDs DID, const ins_2_t* const msg);
    void odom_ins_enu_callback(eDataIDs DID, const ins_2_t* const msg);
    void GPS_info_callback(eDataIDs DID, const gps_sat_t* const msg);
    void mag_callback(eDataIDs DID, const magnetometer_t* const msg);
    void baro_callback(eDataIDs DID, const barometer_t* const msg);
    void preint_IMU_callback(eDataIDs DID, const pimu_t* const msg);
    void strobe_in_time_callback(eDataIDs DID, const strobe_in_time_t* const msg);
    // void diagnostics_callback();
    void GPS_pos_callback(eDataIDs DID, const gps_pos_t* const msg);
    void GPS_vel_callback(eDataIDs DID, const gps_vel_t* const msg);
    void GPS_raw_callback(eDataIDs DID, const gps_raw_t* const msg);
    void GPS_obs_callback(eDataIDs DID, const obsd_t* const msg, int nObs);
    void GPS_obs_bundle_timer_callback();
    void GPS_eph_callback(eDataIDs DID, const eph_t* const msg);
    void GPS_geph_callback(eDataIDs DID, const geph_t* const msg);
    void RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t* const msg);
    void RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t* const msg);

    float diagnostic_ar_ratio_, diagnostic_differential_age_, diagnostic_heading_base_to_rover_;
    uint diagnostic_fix_type_;

    struct DID_INS_1_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::DIDINS1>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct DID_INS_2_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::DIDINS2>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct DID_INS_4_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::DIDINS4>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct INL2_states_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::INL2States>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct odom_stream_t {
        bool enabled = false;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct IMU_stream_t {
        bool enabled = false;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct mag_stream_t {
        bool enabled = false;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct baro_stream_t {
        bool enabled = false;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct preint_IMU_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::PreIntIMU>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct diagnostics_stream_t {
        bool enabled = false;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct GPS_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GPS>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct GPS_info_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GPSInfo>::SharedPtr pub;
        int period_multiple = 1;
    };

    struct GPS_raw_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GNSSObsVec>::SharedPtr pub;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GNSSEphemeris>::SharedPtr pub2;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GlonassEphemeris>::SharedPtr pub3;
        int period_multiple = 1;
    };

    struct GPS_base_raw_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GlonassEphemeris>::SharedPtr pub;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GNSSEphemeris>::SharedPtr pub2;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::GlonassEphemeris>::SharedPtr pub3;
        int period_multiple = 1;
    };

    struct RTK_stream_t {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::RTKInfo>::SharedPtr pub;
        rclcpp::Publisher<inertial_sense_ros_humble_msgs::msg::RTKRel>::SharedPtr pub2;
        int period_multiple = 1;
    };

    struct NavSatFix_stream_t {
        bool enabled = false;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub;
        int period_multiple = 1;
    };

    DID_INS_1_stream_t DID_INS_1_;
    DID_INS_2_stream_t DID_INS_2_;
    DID_INS_4_stream_t DID_INS_4_;
    INL2_states_stream_t INL2_states_;
    odom_stream_t odom_ins_ned_;
    odom_stream_t odom_ins_ecef_;
    odom_stream_t odom_ins_enu_;
    IMU_stream_t IMU_;
    mag_stream_t mag_;
    baro_stream_t baro_;
    preint_IMU_stream_t preint_IMU_;
    diagnostics_stream_t diagnostics_;
    GPS_stream_t GPS1_;
    GPS_info_stream_t GPS1_info_;
    GPS_raw_stream_t GPS1_raw_;
    GPS_stream_t GPS2_;
    GPS_info_stream_t GPS2_info_;
    GPS_raw_stream_t GPS2_raw_;
    GPS_base_raw_stream_t GPS_base_raw_;
    RTK_stream_t RTK_pos_;
    RTK_stream_t RTK_cmp_;
    NavSatFix_stream_t NavSatFix_;

    bool NavSatFixConfigured     = false;
    int gps_raw_period_multiple  = 1;
    int gps_info_period_multiple = 1;

    bool ins1Streaming_          = false;
    bool ins2Streaming_          = false;
    bool ins4Streaming_          = false;
    bool inl2StatesStreaming_    = false;
    bool insCovarianceStreaming_ = false;
    bool magStreaming_           = false;
    bool baroStreaming_          = false;
    bool preintImuStreaming_     = false;
    bool imuStreaming_           = false;
    bool strobeInStreaming_      = false;
    bool diagnosticsStreaming_   = false;
    bool gps1PosStreaming_       = false;
    bool gps1VelStreaming_       = false;
    bool gps2PosStreaming_       = false;
    bool gps2VelStreaming_       = false;
    bool gps1RawStreaming_       = false;
    bool gps2RawStreaming_       = false;
    bool gps1InfoStreaming_      = false;
    bool gps2InfoStreaming_      = false;
    bool rtkPosMiscStreaming_    = false;
    bool rtkPosRelStreaming_     = false;
    bool rtkCmpMiscStreaming_    = false;
    bool rtkCmpRelStreaming_     = false;
    bool data_streams_enabled_   = false;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mag_cal_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr multi_mag_cal_srv_;
    rclcpp::Service<inertial_sense_ros_humble_msgs::srv::Firmwareupdate>::SharedPtr firmware_update_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr refLLA_set_current_srv_;
    rclcpp::Service<inertial_sense_ros_humble_msgs::srv::Refllaupdate>::SharedPtr refLLA_set_value_srv_;

    bool set_current_position_as_refLLA(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
    bool set_refLLA_to_value(inertial_sense_ros_humble_msgs::srv::Refllaupdate::Request& req,
                             inertial_sense_ros_humble_msgs::srv::Refllaupdate::Response& res);
    bool perform_mag_cal_srv_callback(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
    bool perform_multi_mag_cal_srv_callback(std_srvs::srv::Trigger::Request& req,
                                            std_srvs::srv::Trigger::Response& res);
    bool update_firmware_srv_callback(inertial_sense_ros_humble_msgs::srv::Firmwareupdate::Request& req,
                                      inertial_sense_ros_humble_msgs::srv::Firmwareupdate::Response& res);

    void publishGPS1();
    void publishGPS2();

    enum PositionCovarianceType
    {
        COVARIANCE_TYPE_UNKNOWN        = 0,
        COVARIANCE_TYPE_APPROXIMATED   = 1,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
        COVARIANCE_TYPE_KNOWN          = 3
    };

    enum NavSatFixStatusFixType
    {
        STATUS_NO_FIX   = -1, // unable to fix position
        STATUS_FIX      = 0,  // unaugmented fix
        STATUS_SBAS_FIX = 1,  // with satellite-based augmentation
        STATUS_GBAS_FIX = 2   // with ground-based augmentation
    };

    enum NavSatFixService
    {
        SERVICE_GPS     = 0x1,
        SERVICE_GLONASS = 0x2,
        SERVICE_COMPASS = 0x4, // includes BeiDou.
        SERVICE_GALILEO = 0x8
    };

    rclcpp::Time ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek);
    rclcpp::Time ros_time_from_start_time(const double time);
    rclcpp::Time ros_time_from_tow(const double tow);
    double tow_from_ros_time(const rclcpp::Time& rt);
    rclcpp::Time ros_time_from_gtime(const uint64_t sec, double subsec);

    double GPS_towOffset_    = 0;
    uint64_t GPS_week_       = 0;
    double INS_local_offset_ = 0.0;
    bool got_first_message_  = false;
    void LD2Cov(const float* LD, float* Cov, int width);
    void rotMatB2R(const ixVector4 quat, ixMatrix3 R);
    void transform_6x6_covariance(float Pout[36], float Pin[36], ixMatrix3 R1, ixMatrix3 R2);

    // Data to hold on to in between callbacks
    double lla_[3];
    double ecef_[3];
    sensor_msgs::msg::Imu imu_msg;
    nav_msgs::msg::Odometry ned_odom_msg;
    nav_msgs::msg::Odometry ecef_odom_msg;
    nav_msgs::msg::Odometry enu_odom_msg;
    sensor_msgs::msg::NavSatFix NavSatFix_msg;
    inertial_sense_ros_humble_msgs::msg::GPS gps1_msg;
    geometry_msgs::msg::Vector3Stamped gps1_velEcef;
    float gps1_sAcc;
    float gps2_sAcc;
    inertial_sense_ros_humble_msgs::msg::GPSInfo gps_info_msg;
    inertial_sense_ros_humble_msgs::msg::GPS gps2_msg;
    geometry_msgs::msg::Vector3Stamped gps2_velEcef;
    inertial_sense_ros_humble_msgs::msg::GPSInfo gps2_info_msg;
    inertial_sense_ros_humble_msgs::msg::INL2States inl2_states_msg;
    inertial_sense_ros_humble_msgs::msg::DIDINS1 did_ins_1_msg;
    inertial_sense_ros_humble_msgs::msg::DIDINS2 did_ins_2_msg;
    inertial_sense_ros_humble_msgs::msg::DIDINS4 did_ins_4_msg;
    inertial_sense_ros_humble_msgs::msg::PreIntIMU preintIMU_msg;

    float poseCov[36], twistCov[36];

    // Connection to the uINS
    InertialSense IS_;

    // Flash parameters
    int navigation_dt_ms_   = 4;
    float insRotation_[3]   = {0, 0, 0};
    float insOffset_[3]     = {0, 0, 0};
    float gps1AntOffset_[3] = {0, 0, 0};
    float gps2AntOffset_[3] = {0, 0, 0};
    double refLla_[3]       = {0, 0, 0};
    float magInclination_   = 0;
    float magDeclination_   = 0;
    int insDynModel_        = INS_DYN_MODEL_AIRBORNE_4G;
    bool refLLA_known       = false;
    int ioConfig_           = 39624800; // F9P RUG2 RTK CMP: 0x025ca060
    float gpsTimeUserDelay_ = 0;
};