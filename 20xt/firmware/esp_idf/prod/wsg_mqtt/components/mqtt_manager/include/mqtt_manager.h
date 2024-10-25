// Should use both types of header guards
#pragma once
#ifndef _MQTT_MANAGER_H_
#define _MQTT_MANAGER_H_

// Need to define functions as extern if compiling for C++
#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"

#define MQTT_MESSAGE_LENGTH 200

typedef struct __attribute__((packed)) mqtt_data {
    uint32_t data_diff_micro;
    uint16_t data_point;
} mqtt_data_t;

typedef struct __attribute__((packed)) mqtt_message {
    uint64_t start_micro;
    uint8_t mac_address[6];
    mqtt_data_t data[MQTT_MESSAGE_LENGTH];
} mqtt_message_t;

typedef struct mqtt_time {
    uint64_t pi_time_micro;
    uint64_t esp_time_micro;
} mqtt_time_t;

esp_err_t mqtt_start(void);

esp_err_t mqtt_update_time(mqtt_time_t* curr_time);

esp_err_t mqtt_send_message(const char* msg, const uint16_t msg_length);

bool mqtt_can_send_received(void);

// Need to define functions as extern if compiling for C++
#ifdef __cplusplus
}
#endif

#endif