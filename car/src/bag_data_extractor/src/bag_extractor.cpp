#include <chrono>
#include <iomanip>
#include <fstream>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class BagExtractor : public rclcpp::Node
{
public:
  BagExtractor()
  : Node("bag_extractor"),
    imu_count_(0),
    imu_batch_(0),
    last_flush_time_(this->now())
  {
    /* ─── parameters ─── */
    batch_size_         = declare_parameter<int>("batch_size",         1000);
    flush_interval_sec_ = declare_parameter<int>("flush_interval_sec",   15);
    stats_interval_sec_ = declare_parameter<int>("stats_interval_sec",    5);

    /* ─── subscriber ─── */
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::SensorDataQoS{}.reliable(),   // depth 10, KEEP_LAST
        std::bind(&BagExtractor::imuCb, this, _1));

    /* ─── periodic statistics ─── */
    stats_timer_ = create_wall_timer(
        std::chrono::seconds(stats_interval_sec_),
        std::bind(&BagExtractor::printStats, this));
  }

private:
  /* ─── callback ─── */
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr m)
  {
    imu_file_ << m->header.stamp.sec << '.' 
              << std::setw(12) << std::setfill('0') << m->header.stamp.nanosec << ','
              << m->orientation.x         << ',' << m->orientation.y         << ','
              << m->orientation.z         << ',' << m->orientation.w         << ','
              << m->angular_velocity.x    << ',' << m->angular_velocity.y    << ','
              << m->angular_velocity.z    << ','
              << m->linear_acceleration.x << ',' << m->linear_acceleration.y << ','
              << m->linear_acceleration.z << '\n';

    flushIfNeeded(++imu_batch_);
    ++imu_count_;
  }

  /* ─── periodic stats ─── */
  void printStats()
  {
    RCLCPP_INFO(get_logger(), "IMU messages written: %lu", imu_count_);
  }

  /* ─── flush helper ─── */
  void flushIfNeeded(int &batch_counter)
  {
    const auto now = this->now();
    if (batch_counter >= batch_size_ ||
        (now - last_flush_time_).seconds() >= flush_interval_sec_) {
      imu_file_.flush();
      batch_counter = 0;
      last_flush_time_ = now;
    }
  }

  /* ─── members ─── */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::ofstream imu_file_{"imu.csv", std::ios::out | std::ios::trunc};

  uint64_t imu_count_;
  int      imu_batch_;
  int      batch_size_;
  int      flush_interval_sec_;
  int      stats_interval_sec_;
  rclcpp::Time last_flush_time_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagExtractor>());
  rclcpp::shutdown();
  return 0;
}
