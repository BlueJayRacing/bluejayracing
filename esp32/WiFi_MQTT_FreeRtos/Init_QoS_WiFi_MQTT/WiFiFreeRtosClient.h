// by Marius Versteegen, 2022

#include <crt_CleanRTOS.h>
#include "QoSWiFiMQTT.h"
#include "xADS1115.h"

#ifndef WIFIFREERTOSCLIENT_H
#define WIFIFREERTOSCLIENT_H

namespace crt
{
  //Class object to create FREERTOS task that sends values to a BLECLIENT
  class sendValue : public Task
	{
	public:
  	sendValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber, char* ssid, char* pswd, uint8_t* ip);
		static void StaticMain(void *pParam);
	private:
		void main();
    QoSWiFiMQTT* mqtt_client;
	};

  //Class object to create FREERTOS task that records values to be sent by sendValue to a BLECLIENT
	class recordValue : public Task
	{
	public:
		static void StaticMain(void *pParam);
		recordValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber);
	private:
		void main();
	};
}

#endif