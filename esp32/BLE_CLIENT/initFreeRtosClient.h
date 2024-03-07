// by Marius Versteegen, 2022

#include <crt_CleanRTOS.h>
#include "ble_client.h"
#include "bleADS1115.h"
// This file demonstrates the use of a single task.

#ifndef initFreeRtosClient.h
#define initFreeRtosClient.h

namespace crt
{
  //Class object to create FREERTOS task that sends values to a BLECLIENT
  class sendValue : public Task
	{
	public:
  	sendValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber, char* address, char* serviceUUID, char* charUUID);
		static void StaticMain(void *pParam);
	private:
		void main();
		BLECLIENT* bleClient;
    xQueueHandle* queuePointer;
	};

  //Class object to create FREERTOS task that records values to be sent by sendValue to a BLECLIENT
	class recordValue : public Task
	{
	public:
		static void StaticMain(void *pParam);
		recordValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber);
	private:
		void main();
    xQueueHandle* queuePointer;
    int count;
	};
}

#endif