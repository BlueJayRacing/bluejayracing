#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <cmath>
#include <mutex>
#include <vector>
#include <iomanip> // For setprecision

#include "rclcpp/rclcpp.hpp"
#include "inertial_sense_ros_humble_msgs/msg/didins4.hpp"
#include "inertial_sense_ros_humble_msgs/msg/gps.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BagExtractor : public rclcpp::Node
{
public:
  BagExtractor()
  : Node("bag_extractor"),
    didins_count_(0),
    gps1_count_(0),
    gps2_count_(0),
    didins_batch_count_(0),
    gps1_batch_count_(0),
    gps2_batch_count_(0),
    last_stats_time_(this->now()),
    last_flush_time_(this->now())
  {
    // Declare parameters
    this->declare_parameter("batch_size", 1000);
    this->declare_parameter("stats_interval_sec", 5);
    this->declare_parameter("flush_interval_sec", 5);
    this->declare_parameter("max_queue_size", 1000);
    
    // Get parameters
    batch_size_ = this->get_parameter("batch_size").as_int();
    stats_interval_sec_ = this->get_parameter("stats_interval_sec").as_int();
    flush_interval_sec_ = this->get_parameter("flush_interval_sec").as_int();
    max_queue_size_ = this->get_parameter("max_queue_size").as_int();
    
    // Create subscribers with large queue sizes
    rclcpp::QoS qos(max_queue_size_);
    qos.reliable();
    
    didins4_sub_ = this->create_subscription<inertial_sense_ros_humble_msgs::msg::DIDINS4>(
      "/DID_INS_4", qos, std::bind(&BagExtractor::didins4Callback, this, _1));
    
    gps1_sub_ = this->create_subscription<inertial_sense_ros_humble_msgs::msg::GPS>(
      "/gps1", qos, std::bind(&BagExtractor::gps1Callback, this, _1));
    
    gps2_sub_ = this->create_subscription<inertial_sense_ros_humble_msgs::msg::GPS>(
      "/gps2", qos, std::bind(&BagExtractor::gps2Callback, this, _1));
    
    // Open CSV files
    didins_file_.open("didins4_data.csv");
    if (didins_file_.is_open()) {
      didins_file_ << "timestamp_ns,week,time_of_week,ecef_x,ecef_y,ecef_z,abs_velocity,vel_x,vel_y,vel_z" << std::endl;
      didins_file_.flush();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open DIDINS4 CSV file");
    }
    
    gps1_file_.open("gps1_data.csv");
    if (gps1_file_.is_open()) {
      gps1_file_ << "timestamp_ns,week,latitude,longitude,altitude,pos_ecef_x,pos_ecef_y,pos_ecef_z,vel_ecef_x,vel_ecef_y,vel_ecef_z" << std::endl;
      gps1_file_.flush();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GPS1 CSV file");
    }
    
    gps2_file_.open("gps2_data.csv");
    if (gps2_file_.is_open()) {
      gps2_file_ << "timestamp_ns,week,latitude,longitude,altitude,pos_ecef_x,pos_ecef_y,pos_ecef_z,vel_ecef_x,vel_ecef_y,vel_ecef_z" << std::endl;
      gps2_file_.flush();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GPS2 CSV file");
    }
    
    // Create timers
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(stats_interval_sec_), std::bind(&BagExtractor::statsCallback, this));
    
    flush_timer_ = this->create_wall_timer(
      std::chrono::seconds(flush_interval_sec_), std::bind(&BagExtractor::flushCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Separate CSV extractor initialized with:");
    RCLCPP_INFO(this->get_logger(), " - Batch size: %d", batch_size_);
    RCLCPP_INFO(this->get_logger(), " - Stats interval: %d sec", stats_interval_sec_);
    RCLCPP_INFO(this->get_logger(), " - Flush interval: %d sec", flush_interval_sec_);
    RCLCPP_INFO(this->get_logger(), " - Max queue size: %d", max_queue_size_);
  }

  ~BagExtractor()
  {
    // Flush any remaining data
    flushAllFiles();
    
    // Close all files
    if (didins_file_.is_open()) {
      didins_file_.close();
      RCLCPP_INFO(this->get_logger(), "DIDINS4 CSV file closed");
    }
    
    if (gps1_file_.is_open()) {
      gps1_file_.close();
      RCLCPP_INFO(this->get_logger(), "GPS1 CSV file closed");
    }
    
    if (gps2_file_.is_open()) {
      gps2_file_.close();
      RCLCPP_INFO(this->get_logger(), "GPS2 CSV file closed");
    }
    
    // Print final statistics
    printStatistics();
  }

private:
  // Convert GPS time (week + time of week) to nanoseconds since epoch
  uint64_t gpsTimeToUnixNano(uint32_t week, double time_of_week)
  {
    // GPS epoch started on January 6, 1980 at 00:00:00 UTC
    // UNIX epoch started on January 1, 1970 at 00:00:00 UTC
    // 315964800 is the number of seconds between these two epochs
    
    // As of 2024, there are 18 leap seconds between GPS time and UTC
    const int GPS_LEAP_SECONDS = 18;
    
    // Convert to nanoseconds
    uint64_t gps_nanoseconds = static_cast<uint64_t>(week) * 604800000000000LL + 
                              static_cast<uint64_t>(time_of_week * 1000000000.0);
    
    // Add offset from GPS epoch to UNIX epoch (in nanoseconds)
    uint64_t unix_nanoseconds = gps_nanoseconds + 315964800000000000LL - 
                               (GPS_LEAP_SECONDS * 1000000000LL);
    
    return unix_nanoseconds;
  }

  // DID_INS_4 message callback
  void didins4Callback(const inertial_sense_ros_humble_msgs::msg::DIDINS4::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(didins_mutex_);
    
    // Convert GPS time to Unix time in nanoseconds
    uint64_t timestamp_ns = gpsTimeToUnixNano(msg->week, msg->time_of_week);
    
    // Calculate absolute velocity
    double abs_velocity = std::sqrt(
      msg->ve[0] * msg->ve[0] + 
      msg->ve[1] * msg->ve[1] + 
      msg->ve[2] * msg->ve[2]);
    
    // Write to CSV file
    didins_file_ << timestamp_ns << ","
               << msg->week << ","
               << std::fixed << std::setprecision(9) << msg->time_of_week << ","
               << std::setprecision(6)
               << msg->ecef[0] << ","
               << msg->ecef[1] << ","
               << msg->ecef[2] << ","
               << abs_velocity << ","
               << msg->ve[0] << ","
               << msg->ve[1] << ","
               << msg->ve[2] << "\n";
    
    didins_count_++;
    didins_batch_count_++;
    
    // Flush if batch size reached
    if (didins_batch_count_ >= batch_size_) {
      didins_file_.flush();
      didins_batch_count_ = 0;
    }
  }

  // GPS1 message callback
  void gps1Callback(const inertial_sense_ros_humble_msgs::msg::GPS::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(gps1_mutex_);
    
    // Convert GPS time to Unix time in nanoseconds
    uint64_t timestamp_ns = gpsTimeToUnixNano(msg->week, msg->header.stamp.sec + msg->header.stamp.nanosec/1e9);
    
    // Write to CSV file
    gps1_file_ << timestamp_ns << ","
              << msg->week << ","
              << std::fixed << std::setprecision(10)
              << msg->latitude << ","
              << msg->longitude << ","
              << std::setprecision(6)
              << msg->altitude << ","
              << msg->pos_ecef.x << ","
              << msg->pos_ecef.y << ","
              << msg->pos_ecef.z << ","
              << msg->vel_ecef.x << ","
              << msg->vel_ecef.y << ","
              << msg->vel_ecef.z << "\n";
    
    gps1_count_++;
    gps1_batch_count_++;
    
    // Flush if batch size reached
    if (gps1_batch_count_ >= batch_size_) {
      gps1_file_.flush();
      gps1_batch_count_ = 0;
    }
  }

  // GPS2 message callback
  void gps2Callback(const inertial_sense_ros_humble_msgs::msg::GPS::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(gps2_mutex_);
    
    // Convert GPS time to Unix time in nanoseconds
    uint64_t timestamp_ns = gpsTimeToUnixNano(msg->week, msg->header.stamp.sec + msg->header.stamp.nanosec/1e9);
    
    // Write to CSV file
    gps2_file_ << timestamp_ns << ","
              << msg->week << ","
              << std::fixed << std::setprecision(10)
              << msg->latitude << ","
              << msg->longitude << ","
              << std::setprecision(6)
              << msg->altitude << ","
              << msg->pos_ecef.x << ","
              << msg->pos_ecef.y << ","
              << msg->pos_ecef.z << ","
              << msg->vel_ecef.x << ","
              << msg->vel_ecef.y << ","
              << msg->vel_ecef.z << "\n";
    
    gps2_count_++;
    gps2_batch_count_++;
    
    // Flush if batch size reached
    if (gps2_batch_count_ >= batch_size_) {
      gps2_file_.flush();
      gps2_batch_count_ = 0;
    }
  }

  // Flush all files to disk
  void flushAllFiles()
  {
    {
      std::lock_guard<std::mutex> lock(didins_mutex_);
      if (didins_file_.is_open()) {
        didins_file_.flush();
        didins_batch_count_ = 0;
      }
    }
    
    {
      std::lock_guard<std::mutex> lock(gps1_mutex_);
      if (gps1_file_.is_open()) {
        gps1_file_.flush();
        gps1_batch_count_ = 0;
      }
    }
    
    {
      std::lock_guard<std::mutex> lock(gps2_mutex_);
      if (gps2_file_.is_open()) {
        gps2_file_.flush();
        gps2_batch_count_ = 0;
      }
    }
  }

  // Timer callback for flushing files
  void flushCallback()
  {
    flushAllFiles();
    
    auto now = this->now();
    double elapsed = (now - last_flush_time_).seconds();
    last_flush_time_ = now;
    
    RCLCPP_INFO(this->get_logger(), "Flushed all files to disk after %.1f seconds", elapsed);
  }

  // Timer callback for printing statistics
  void statsCallback()
  {
    printStatistics();
  }

  // Print processing statistics
  void printStatistics()
  {
    auto now = this->now();
    double elapsed = (now - last_stats_time_).seconds();
    
    if (elapsed <= 0.1) {
      return; // Avoid division by near-zero
    }
    
    // Calculate rates
    double didins_rate = static_cast<double>(didins_count_) / elapsed;
    double gps1_rate = static_cast<double>(gps1_count_) / elapsed;
    double gps2_rate = static_cast<double>(gps2_count_) / elapsed;
    double total_rate = didins_rate + gps1_rate + gps2_rate;
    
    RCLCPP_INFO(this->get_logger(),
               "Stats: %.1f msgs/sec (DIDINS: %.1f, GPS1: %.1f, GPS2: %.1f) | "
               "Batches: DIDINS=%d, GPS1=%d, GPS2=%d | Total written: DIDINS=%d, GPS1=%d, GPS2=%d",
               total_rate, didins_rate, gps1_rate, gps2_rate,
               didins_batch_count_, gps1_batch_count_, gps2_batch_count_,
               didins_total_, gps1_total_, gps2_total_);
    
    // Update total counts before resetting message counters
    didins_total_ += didins_count_;
    gps1_total_ += gps1_count_;
    gps2_total_ += gps2_count_;
    
    // Reset counters
    didins_count_ = 0;
    gps1_count_ = 0;
    gps2_count_ = 0;
    last_stats_time_ = now;
  }
  
  // Subscribers
  rclcpp::Subscription<inertial_sense_ros_humble_msgs::msg::DIDINS4>::SharedPtr didins4_sub_;
  rclcpp::Subscription<inertial_sense_ros_humble_msgs::msg::GPS>::SharedPtr gps1_sub_;
  rclcpp::Subscription<inertial_sense_ros_humble_msgs::msg::GPS>::SharedPtr gps2_sub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  
  // File streams
  std::ofstream didins_file_;
  std::ofstream gps1_file_;
  std::ofstream gps2_file_;
  
  // Mutexes for thread safety
  std::mutex didins_mutex_;
  std::mutex gps1_mutex_;
  std::mutex gps2_mutex_;
  
  // Parameters
  int batch_size_;
  int stats_interval_sec_;
  int flush_interval_sec_;
  int max_queue_size_;
  
  // Statistics and counters
  int didins_count_;
  int gps1_count_;
  int gps2_count_;
  int didins_batch_count_;
  int gps1_batch_count_;
  int gps2_batch_count_;
  int didins_total_ = 0;
  int gps1_total_ = 0;
  int gps2_total_ = 0;
  rclcpp::Time last_stats_time_;
  rclcpp::Time last_flush_time_;
};

int main(int argc, char * argv[])
{
  // Set real-time priority for the process if possible
  #ifdef __linux__
  struct sched_param sp;
  sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
  sched_setscheduler(0, SCHED_FIFO, &sp);
  #endif
  
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<BagExtractor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}