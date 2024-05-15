#ifndef WIFIFREERTOSCLIENT_H
#define WIFIFREERTOSCLIENT_H

#include <crt_CleanRTOS.h>
#include <ESP32Time.h>

#include "../qos_mqtt/qos_mqtt.h"
#include "../ads_1120/ads_1120.h"

namespace crt
{
	class record_value : public Task
	{
	public:
		static void StaticMain(void *p_param);
		record_value(const char *task_name, unsigned int task_priority, unsigned int task_size_bytes, unsigned int task_core_number);

	private:
		void main();
		uint32_t get_rtc_millis(ESP32Time &rtc);
	};

	class send_value : public Task
	{
	public:
		send_value(const char *task_name, unsigned int task_priority, unsigned int task_size_bytes, unsigned int task_core_number, uint8_t *ip_address);
		static void StaticMain(void *p_param);

	private:
		void main();
		void create_publish_topic(char *publish_topic);
		void copy_time_to_message(uint8_t *message, uint32_t time);
		void copy_data_to_message(uint8_t *message, uint16_t *data);
		char *ssid;
		char *pswd;
		uint8_t *broker_ip_address;
	};
}

#endif