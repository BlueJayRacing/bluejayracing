#pragma once

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>

#include "InertialSense.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "inertial_sense_ros_humble_msgs/msg/gps.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gps_info.hpp"
#include "inertial_sense_ros_humble_msgs/msg/pre_int_imu.hpp"
#include "inertial_sense_ros_humble_msgs/msg/rtk_rel.hpp"
#include "inertial_sense_ros_humble_msgs/msg/rtk_info.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gnss_ephemeris.hpp"
#include "inertial_sense_ros_humble_msgs/msg/glonass_ephemeris.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gnss_observation.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gnss_obs_vec.hpp"
#include "inertial_sense_ros_humble_msgs/msg/inl2_states.hpp"
#include "inertial_sense_ros_humble_msgs/msg/didins2.hpp"
#include "inertial_sense_ros_humble_msgs/msg/didins1.hpp"
#include "inertial_sense_ros_humble_msgs/msg/didins4.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
//#include "geometry/xform.h"

# define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
# define LEAP_SECONDS 18 // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
# define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple) \
    IS_.BroadcastBinaryData(DID, __periodmultiple, \
    [this](InertialSense*i, p_data_t* data, int pHandle)\
    { \
       /* ROS_INFO("Got message %d", DID);*/\
       this->__cb_fun(reinterpret_cast<__type*>(data->buf));\
    })


class InertialSenseROS  : public rclcpp::Node//: SerialListener
{
public:
  typedef enum
  {
    NMEA_GPGGA = 0x01,
    NMEA_GPGLL = 0x02,
    NMEA_GPGSA = 0x04,
    NMEA_GPRMC = 0x08,
    NMEA_SER0 = 0x01,
    NMEA_SER1 = 0x02
  } NMEA_message_config_t;
      
  InertialSenseROS();
  void callback(p_data_t* data);
  void update();

  void connect();
  void set_navigation_dt_ms();
  void configure_parameters();
  void configure_rtk();
  void configure_data_streams();
  void configure_ascii_output();
  void start_log();
  
  template<typename T> void set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset);
  template<typename T>  void set_flash_config(std::string param_name, uint32_t offset, T def) __attribute__ ((optimize(0)));
  void get_flash_config();
  void reset_device();
  void flash_config_callback(const nvm_flash_cfg_t* const msg);
  // Serial Port Configuration
  std::string port_;
  int baudrate_;
  bool initialized_;
  bool log_enabled_;

  std::string frame_id_;

  // ROS Stream handling
  // template <typename T>
  // struct ros_stream_t
  // {
  //   bool enabled;
  //   typename rclcpp::Publisher<T>::SharedPtr pub;
  //   typename rclcpp::Publisher<T>::SharedPtr pub2;
  // };

  struct ros_stream_t_IMU
  {
    bool enabled;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub2;
  };
  ros_stream_t_IMU IMU_;

  struct ros_stream_t_baro
  {
    bool enabled;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub2;
  };
  ros_stream_t_baro baro_;

  struct ros_stream_t_diag
  {
    bool enabled;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub2;
  };
  ros_stream_t_diag diagnostics_;

  //ros_stream_t IMU_;
  void IMU_callback(const dual_imu_t* const msg);

  //ros_stream_t baro_;
  void baro_callback(const barometer_t* const msg);

  //ros_stream_t dt_vel_;
  void preint_IMU_callback(const preintegrated_imu_t * const msg);

  //ros_stream_t diagnostics_;
  void diagnostics_callback(); // It had parameter: const rclcpp::TimerEvent& event
  //template<typename DurationRepT , typename DurationT , typename CallbackT>
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  float diagnostic_ar_ratio_, diagnostic_differential_age_, diagnostic_heading_base_to_rover_;

  // rclcpp::Service mag_cal_srv_;
  // rclcpp::Service multi_mag_cal_srv_;
  //rclcpp::Service<inertial_sense_ros::srv::FirmwareUpdate> firmware_update_srv_;
  // rclcpp::Service<inertial_sense_ros::srv::refLLAUpdate>  refLLA_set_current_srv_;
  // rclcpp::Service<inertial_sense_ros::srv::refLLAUpdate>  refLLA_set_value_srv_;
  // bool set_current_position_as_refLLA(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response & res);
  // bool set_refLLA_to_value(inertial_sense_ros::refLLAUpdate::Request &req, inertial_sense_ros::refLLAUpdate::Response &res);
  // bool perform_mag_cal_srv_callback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  // bool perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  // bool update_firmware_srv_callback(inertial_sense_ros::FirmwareUpdate::Request & req, inertial_sense_ros::FirmwareUpdate::Response & res);

  
  /**
   * @brief ros_time_from_week_and_tow
   * Get current ROS time from week and tow
   * @param week Weeks since January 6th, 1980
   * @param timeOfWeek Time of week (since Sunday morning) in seconds, GMT
   * @return equivalent rclcpp::Time
   */
  rclcpp::Time ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek);
  
  /**
   * @brief ros_time_from_start_time
   * @param time - Time since boot up in seconds - Convert to GPS time of week by adding gps.towOffset
   * @return equivalent rclcpp::Time
   */
  rclcpp::Time ros_time_from_start_time(const double time);
  
  /**
   * @brief ros_time_from_tow
   * Get equivalent ros time from tow and internal week counter
   * @param tow Time of Week (seconds)
   * @return equivalent rclcpp::Time
   */
  rclcpp::Time ros_time_from_tow(const double tow);

  double tow_from_ros_time(const rclcpp::Time& rt);
  rclcpp::Time ros_time_from_gtime(const uint64_t sec, double subsec);

  double GPS_towOffset_ = 0; // The offset between GPS time-of-week and local time on the uINS
                             //  If this number is 0, then we have not yet got a fix
  uint64_t GPS_week_ = 0; // Week number to start of GPS_towOffset_ in GPS time
  // Time sync variables
  double INS_local_offset_ = 0.0; // Current estimate of the uINS start time in ROS time seconds
  bool got_first_message_ = false; // Flag to capture first uINS start time guess

  // Data to hold on to in between callbacks
  double lla_[3];
  double ecef_[3];
  sensor_msgs::msg::Imu imu1_msg, imu2_msg;
  nav_msgs::msg::Odometry odom_msg;
  //inertial_sense_ros::msg::GPS gps_msg; 
  geometry_msgs::msg::Vector3Stamped gps_velEcef;
  //inertial_sense_ros::msg::GPSInfo gps_info_msg;
  //inertial_sense_ros::msg::INL2States inl2_states_msg;

  // Connection to the uINS
  InertialSense IS_;
};