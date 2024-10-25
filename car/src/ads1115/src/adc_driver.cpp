// adc_driver.cpp
#include <adc_driver/adc_driver.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace adc_driver
{

ADCDriver::ADCDriver() : Node("adc_driver")
{
    int fd   = open("/dev/i2c-1", O_RDWR);
    int fd6  = open("/dev/i2c-6", O_RDWR);
    adcs_[4] = ADC(fd6, 0, false);
    for (int i = 0; i < 4; i++) {
        adcs_[i] = ADC(fd, i, false);
    }

    publisher_ = create_publisher<baja_msgs::msg::Observation>("adc_data", 10);

    channels_[0][0] = baja_msgs::msg::AnalogChannel::SHOCK_LEN_FRONT_RIGHT;
    channels_[0][1] = baja_msgs::msg::AnalogChannel::SHOCK_LEN_REAR_RIGHT;
    channels_[0][2] = baja_msgs::msg::AnalogChannel::BRAKE_PRESSURE_FRONT;
    channels_[0][3] = baja_msgs::msg::AnalogChannel::AXLE_RPM_REAR;
    channels_[0][4] = baja_msgs::msg::AnalogChannel::AXLE_RPM_FRONT_RIGHT;
    channels_[1][0] = baja_msgs::msg::AnalogChannel::SHOCK_LEN_FRONT_LEFT;
    channels_[1][1] = baja_msgs::msg::AnalogChannel::SHOCK_LEN_REAR_LEFT;
    channels_[1][2] = baja_msgs::msg::AnalogChannel::BRAKE_PRESSURE_REAR;
    channels_[1][3] = baja_msgs::msg::AnalogChannel::TACHOMETER;
    channels_[1][4] = baja_msgs::msg::AnalogChannel::AXLE_RPM_FRONT_LEFT;

    timer_ = create_wall_timer(50ms, std::bind(&ADCDriver::read_and_publish_data, this));
}

void ADCDriver::read_and_publish_data()
{
    auto begin = std::chrono::high_resolution_clock::now();
    baja_msgs::msg::Observation obs;

    for (size_t i = 0; i < 5; i++) {
        std::vector<double> data           = adcs_[i].read();
        auto log                           = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = log - begin;
        // std::cout << data[0] << std::endl;
        adcs_[i].swap_channel(1);

        auto analog_channel                  = baja_msgs::msg::AnalogChannel();
        analog_channel.channel_type          = channels_[0][i];
        analog_channel.encoded_analog_points = data[0];
        obs.analog_ch                        = std::vector<baja_msgs::msg::AnalogChannel>({analog_channel});

        obs.timestamp.ts = diff.count();
        publisher_->publish(obs);
    }

    for (size_t i = 0; i < 5; i++) {
        std::vector<double> data           = adcs_[i].read();
        auto log                           = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = log - begin;
        // std::cout << data[0] << std::endl;
        adcs_[i].swap_channel(0);

        auto analog_channel                  = baja_msgs::msg::AnalogChannel();
        analog_channel.channel_type          = channels_[1][i];
        analog_channel.encoded_analog_points = data[0];
        obs.analog_ch                        = std::vector<baja_msgs::msg::AnalogChannel>({analog_channel});

        obs.timestamp.ts = diff.count();
        publisher_->publish(obs);
    }

    // std::cout << std::endl;
}

std::string ADCDriver::serializeDoubleToBinaryString(double value)
{
    const unsigned char* p = reinterpret_cast<const unsigned char*>(&value);
    return std::string(p, p + sizeof(double));
}

} // namespace adc_driver