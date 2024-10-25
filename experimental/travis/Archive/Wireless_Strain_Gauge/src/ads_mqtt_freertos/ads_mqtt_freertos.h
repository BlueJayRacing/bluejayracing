#ifndef WIFIFREERTOSCLIENT_H
#define WIFIFREERTOSCLIENT_H

#include <ESP32Time.h>
#include <crt_CleanRTOS.h>

#include "../ads_1120/ads_1120.h"
#include "../qos_mqtt/qos_mqtt.h"

namespace crt
{
// Class object to create FREERTOS task that sends values to a BLECLIENT
class send_value : public Task {
  public:
    send_value(const char* task_name, unsigned int task_priority, unsigned int task_size_bytes,
               unsigned int task_core_number, uint8_t* ip_address);
    static void StaticMain(void* p_param);

  private:
    void main();
    char* ssid;
    char* pswd;
    uint8_t* broker_ip_address;
};

// Class object to create FREERTOS task that records values to be sent by sendValue to a BLECLIENT
class record_value : public Task {
  public:
    static void StaticMain(void* p_param);
    record_value(const char* task_name, unsigned int task_priority, unsigned int task_size_bytes,
                 unsigned int task_core_number);

  private:
    void main();
};
} // namespace crt

#endif