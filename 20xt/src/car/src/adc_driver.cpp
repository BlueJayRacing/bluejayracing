#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>

#include "ipc_config.h"
#include "baja_live_comm.pb.h"
#include "adc.hpp"

#include <chrono>


using std::chrono::duration_cast;

#define NUMADC 4

std::string serializeDoubleToBinaryString(double value) {
    // Cast the address of 'value' to a pointer to 'unsigned char'
    const unsigned char* p = reinterpret_cast<const unsigned char*>(&value);
    // Construct a string from the binary data of 'value'
    return std::string(p, p + sizeof(double));
}

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  auto begin = std::chrono::high_resolution_clock::now();


  int fd = open("/dev/i2c-1", O_RDWR);
  int fd6 = open("/dev/i2c-6", O_RDWR);

  ADC adcs[NUMADC+1];
  adcs[4] = ADC(fd6, 0, false);
  for (int i = 0; i < NUMADC; i++) {
    adcs[i] = ADC(fd, i, false);
  }

  // Open the message queue, return if it fails
  mqd_t tx_queue = BajaIPC::open_queue(CarIPC::ADC_DRIVER_TO_BROKER_QUEUE, false);
  if (tx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  AnalogChannel_ChannelType channels[2][5] = {{
      AnalogChannel::SHOCK_LEN_FRONT_RIGHT,
      AnalogChannel::SHOCK_LEN_REAR_RIGHT,
      AnalogChannel::BRAKE_PRESSURE_FRONT,
      AnalogChannel::AXLE_RPM_REAR,
      AnalogChannel::AXLE_RPM_FRONT_RIGHT
      
    },{
      AnalogChannel::SHOCK_LEN_FRONT_LEFT,
      AnalogChannel::SHOCK_LEN_REAR_LEFT,
      AnalogChannel::BRAKE_PRESSURE_REAR,
      AnalogChannel::TACHOMETER,
      AnalogChannel::AXLE_RPM_FRONT_LEFT
      
    }
  };

  return -1;

  while (true) {
    Observation obs;
    std::string msg;

    for (size_t i = 0; i < NUMADC+1; i++) {
    	std::vector<double> data = adcs[i].read();
      auto log = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff = log - begin;
      std::cout << data[0] << std::endl; 
      adcs[i].swap_channel(1);
      AnalogChannel* analog_channel = new AnalogChannel();
      analog_channel->set_channel_type(channels[0][i]);
      analog_channel->set_encoded_analog_points(serializeDoubleToBinaryString(data[0]));

      obs.set_allocated_analog_ch(analog_channel);
      Timestamp* timestamp = new Timestamp();
      timestamp->set_ts(diff.count());
      obs.set_allocated_timestamp(timestamp);
      obs.SerializeToString(&msg);
      BajaIPC::send_message(tx_queue, msg);
    }
    for (size_t i = 0; i<NUMADC+1; i++) {
      std::vector<double> data = adcs[i].read();
      auto log = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff = log - begin;
      std::cout << data[0] << std::endl;
      adcs[i].swap_channel(0);
      AnalogChannel* analog_channel = new AnalogChannel();
      analog_channel->set_channel_type(channels[1][i]);
      analog_channel->set_encoded_analog_points(serializeDoubleToBinaryString(data[0]));

      obs.set_allocated_analog_ch(analog_channel);
      obs.SerializeToString(&msg);
      Timestamp* timestamp = new Timestamp();
      timestamp->set_ts(diff.count());
      obs.set_allocated_timestamp(timestamp);
      BajaIPC::send_message(tx_queue, msg);
    }
    std::cout << std::endl;
}
}
