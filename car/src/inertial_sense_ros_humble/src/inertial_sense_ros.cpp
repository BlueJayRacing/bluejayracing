#include "inertial_sense_ros_humble/inertial_sense_ros.h"
#include <chrono>
#include <stddef.h>
#include <unistd.h>
// #include <ros/console.h>
#include <ISPose.h>

InertialSenseROS::InertialSenseROS() : Node("inertial_sense")
{
  connect();
  // set_navigation_dt_ms();

  /// Start Up ROS service servers
  // refLLA_set_current_srv_ = nh_.advertiseService("set_refLLA_current", &InertialSenseROS::set_current_position_as_refLLA, this);
  // refLLA_set_value_srv_ = nh_.advertiseService("set_refLLA_value", &InertialSenseROS::set_refLLA_to_value, this);
  // mag_cal_srv_ = nh_.advertiseService("single_axis_mag_cal", &InertialSenseROS::perform_mag_cal_srv_callback, this);
  // multi_mag_cal_srv_ = nh_.advertiseService("multi_axis_mag_cal", &InertialSenseROS::perform_multi_mag_cal_srv_callback, this);
  // firmware_update_srv_ = node->create_service<inertial_sense_ros::srv::FirmwareUpdate>("firmware_update", &InertialSenseROS::update_firmware_srv_callback);

  configure_parameters();
  configure_data_streams();
  this->declare_parameter<bool>("enable_log", false);
  this->get_parameter("enable_log", log_enabled_);
  if (log_enabled_)
  {
    start_log();//start log should always happen last, does not all stop all message streams.
  }

  
//  configure_ascii_output(); //does not work right now

  initialized_ = true;
}


void InertialSenseROS::configure_data_streams()
{
  
  // Set up the IMU ROS stream
  this->declare_parameter<bool>("stream_IMU", true);
  this->get_parameter("stream_IMU", IMU_.enabled);

  //std::cout << "\n\n\n\n\n\n\n\n\n\n stream_GPS: " << GPS_.enabled << "\n\n\n\n\n\n\n\n\n\n\n";
  RCLCPP_INFO(this->get_logger(), "Setting up IMU");
  if (IMU_.enabled)
  {
    IMU_.pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    SET_CALLBACK(DID_DUAL_IMU, dual_imu_t, IMU_callback, 1);
    RCLCPP_INFO(this->get_logger(), "IMU setup");

  }

  // Set up the barometer ROS stream
  this->declare_parameter<bool>("stream_baro", false);
  this->get_parameter("stream_baro", baro_.enabled);
  if (baro_.enabled)
  {
    baro_.pub = this->create_publisher<sensor_msgs::msg::FluidPressure>("baro", 1);
    SET_CALLBACK(DID_BAROMETER, barometer_t, baro_callback, 1);
  }

  // Set up ROS dianostics for rqt_robot_monitor
  this->declare_parameter<bool>("stream_diagnostics", true);
  this->get_parameter("stream_diagnostics", diagnostics_.enabled);
  if (diagnostics_.enabled)
  {
    diagnostics_.pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
    diagnostics_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&InertialSenseROS::diagnostics_callback, this)); // 2 Hz
  }
}

void InertialSenseROS::start_log()
{
  std::string filename = getenv("HOME");
  filename += "/Documents/Inertial_Sense/Logs/" + cISLogger::CreateCurrentTimestamp();
  RCLCPP_INFO_STREAM(this->get_logger(), "Creating log in " << filename << " folder");
  IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_BITS);
}

void InertialSenseROS::configure_ascii_output()
{
  //  int NMEA_rate = nh_private_.param<int>("NMEA_rate", 0);
  //  int NMEA_message_configuration = nh_private_.param<int>("NMEA_configuration", 0x00);
  //  int NMEA_message_ports = nh_private_.param<int>("NMEA_ports", 0x00);
  //  ascii_msgs_t msgs = {};
  //  msgs.options = (NMEA_message_ports & NMEA_SER0) ? RMC_OPTIONS_PORT_SER0 : 0; // output on serial 0
  //  msgs.options |= (NMEA_message_ports & NMEA_SER1) ? RMC_OPTIONS_PORT_SER1 : 0; // output on serial 1
  //  msgs.gpgga = (NMEA_message_configuration & NMEA_GPGGA) ? NMEA_rate : 0;
  //  msgs.gpgll = (NMEA_message_configuration & NMEA_GPGLL) ? NMEA_rate : 0;
  //  msgs.gpgsa = (NMEA_message_configuration & NMEA_GPGSA) ? NMEA_rate : 0;
  //  msgs.gprmc = (NMEA_message_configuration & NMEA_GPRMC) ? NMEA_rate : 0;
  //  IS_.SendData(DID_ASCII_BCAST_PERIOD, (uint8_t*)(&msgs), sizeof(ascii_msgs_t), 0);

}

void InertialSenseROS::connect()
{
  this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  this->get_parameter("port", port_);
  this->declare_parameter<std::string>("frame_id", "body");
  this->get_parameter("frame_id", frame_id_);
  this->declare_parameter<int>("baud", 921600);
  this->get_parameter("baud", baudrate_);
  
  /// Connect to the uINS
  RCLCPP_INFO(this->get_logger(), "Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
  if (! IS_.Open(port_.c_str(), baudrate_))
  {
    RCLCPP_FATAL(this->get_logger(), "inertialsense: Unable to open serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    exit(0);
  }
  else
  {
    // Print if Successful
    RCLCPP_INFO(this->get_logger(), "Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, port_.c_str(), baudrate_);
  }
}

void InertialSenseROS::set_navigation_dt_ms()
{
  // // Make sure the navigation rate is right, if it's not, then we need to change and reset it.
  // int nav_dt_ms = IS_.GetFlashConfig().startupNavDtMs;
  // if (nh_private_.getParam("navigation_dt_ms", nav_dt_ms))
  // {
  //   if (nav_dt_ms != IS_.GetFlashConfig().startupNavDtMs)
  //   {
  //     uint32_t data = nav_dt_ms;
  //     IS_.SendData(DID_FLASH_CONFIG, (uint8_t*)(&data), sizeof(uint32_t), offsetof(nvm_flash_cfg_t, startupNavDtMs));
  //     ROS_INFO("navigation rate change from %dms to %dms, resetting uINS to make change", IS_.GetFlashConfig().startupNavDtMs, nav_dt_ms);
  //     sleep(3);
  //     reset_device();
  //   }
  // }
}

void InertialSenseROS::configure_parameters()
{
  // set_vector_flash_config<float>("INS_rpy_radians", 3, offsetof(nvm_flash_cfg_t, insRotation));
  // set_vector_flash_config<float>("INS_xyz", 3, offsetof(nvm_flash_cfg_t, insOffset));
  // set_vector_flash_config<float>("GPS_ant1_xyz", 3, offsetof(nvm_flash_cfg_t, gps1AntOffset));
  // set_vector_flash_config<float>("GPS_ant2_xyz", 3, offsetof(nvm_flash_cfg_t, gps2AntOffset));
  // set_vector_flash_config<double>("GPS_ref_lla", 3, offsetof(nvm_flash_cfg_t, refLla));

  // set_flash_config<float>("inclination", offsetof(nvm_flash_cfg_t, magInclination), 0.0f);
  // set_flash_config<float>("declination", offsetof(nvm_flash_cfg_t, magDeclination), 0.0f);
  // set_flash_config<int>("dynamic_model", offsetof(nvm_flash_cfg_t, insDynModel), 8);
  set_flash_config<int>("ser1_baud_rate", offsetof(nvm_flash_cfg_t, ser1BaudRate), 921600);
  RCLCPP_INFO(this->get_logger(), "Baudrate set");
}

// template <typename T>
// void InertialSenseROS::set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset){
//   std::vector<double> tmp(size,0);
//   T v[size];
//   if (nh_private_.hasParam(param_name))
//     nh_private_.getParam(param_name, tmp);
//   for (int i = 0; i < size; i++)
//   {
//     v[i] = tmp[i];
//   }
  
//   IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&v), sizeof(v), offset);
//   IS_.GetFlashConfig() = IS_.GetFlashConfig();
// }

template <typename T>
void InertialSenseROS::set_flash_config(std::string param_name, uint32_t offset, T def)
{
  T tmp;
  this->declare_parameter<T>(param_name, def);
  this->get_parameter(param_name, tmp);
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&tmp), sizeof(T), offset);
}

void InertialSenseROS::IMU_callback(const dual_imu_t* const msg)
{
  imu1_msg.header.stamp = imu2_msg.header.stamp = ros_time_from_start_time(msg->time);
  imu1_msg.header.frame_id = imu2_msg.header.frame_id = frame_id_;

  imu1_msg.angular_velocity.x = msg->I[0].pqr[0];
  imu1_msg.angular_velocity.y = msg->I[0].pqr[1];
  imu1_msg.angular_velocity.z = msg->I[0].pqr[2];
  imu1_msg.linear_acceleration.x = msg->I[0].acc[0];
  imu1_msg.linear_acceleration.y = msg->I[0].acc[1];
  imu1_msg.linear_acceleration.z = msg->I[0].acc[2];

  //  imu2_msg.angular_velocity.x = msg->I[1].pqr[0];
  //  imu2_msg.angular_velocity.y = msg->I[1].pqr[1];
  //  imu2_msg.angular_velocity.z = msg->I[1].pqr[2];
  //  imu2_msg.linear_acceleration.x = msg->I[1].acc[0];
  //  imu2_msg.linear_acceleration.y = msg->I[1].acc[1];
  //  imu2_msg.linear_acceleration.z = msg->I[1].acc[2];

  if (IMU_.enabled)
  {
    IMU_.pub->publish(imu1_msg);
    //    IMU_.pub2.publish(imu2_msg);
  }
}


void InertialSenseROS::update()
{
	IS_.Update();
}


void InertialSenseROS::baro_callback(const barometer_t * const msg)
{
  sensor_msgs::msg::FluidPressure baro_msg;
  baro_msg.header.stamp = ros_time_from_start_time(msg->time);
  baro_msg.header.frame_id = frame_id_;
  baro_msg.fluid_pressure = msg->bar;
  baro_msg.variance = msg-> barTemp;

  baro_.pub->publish(baro_msg);
}


void InertialSenseROS::diagnostics_callback()
{
  // Create diagnostic objects
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  // CNO mean
  diagnostic_msgs::msg::DiagnosticStatus cno_mean;
  cno_mean.name = "CNO Mean";
  cno_mean.level =  diagnostic_msgs::msg::DiagnosticStatus::OK;
  //cno_mean.message = std::to_string(gps_msg.cno);
  diag_array.status.push_back(cno_mean);

  diagnostics_.pub->publish(diag_array);
}

void InertialSenseROS::reset_device()
{
  // send reset command
  system_command_t reset_command;
  reset_command.command = 99;
  reset_command.invCommand = ~reset_command.command;
  IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t*>(&reset_command), sizeof(system_command_t), 0);
  sleep(1);
}

// bool InertialSenseROS::update_firmware_srv_callback(std::shared_ptr<inertial_sense_ros::srv::FirmwareUpdate::Request> req, std::shared_ptr<inertial_sense_ros::srv::FirmwareUpdate::Request> res)
// {
//   IS_.Close();
//   vector<InertialSense::bootloader_result_t> results = IS_.BootloadFile("*", req->filename, 921600);
//   if (!results[0].error.empty())
//   {
//     res->success = false;
//     res->message = results[0].error;
//     return false;
//   }
//   IS_.Open(port_.c_str(), baudrate_);
//   return true;
// }

rclcpp::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
  rclcpp::Time rostime(0, 0);
  
  
  // Otherwise, estimate the uINS boot time and offset the messages
  if (!got_first_message_)
  {
    got_first_message_ = true;
    INS_local_offset_ = this->now().seconds() - time;
  }
  else // low-pass filter offset to account for drift
  {
    double y_offset = this->now().seconds() - time;
    INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
  }
  // Publish with ROS time
  rostime = rclcpp::Time(INS_local_offset_ + time);
  
  return rostime;
}