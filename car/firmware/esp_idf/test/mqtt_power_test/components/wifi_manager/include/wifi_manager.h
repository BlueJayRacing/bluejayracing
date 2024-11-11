// Should use both types of header guards
#pragma once
#ifndef _COMPONENT_TEMPLATE_H_
#define _COMPONENT_TEMPLATE_H_

// Need to define functions as extern if compiling for C++
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

esp_err_t wifi_start(void);

esp_err_t wifi_connect(void);

esp_err_t wifi_stop(void);

esp_err_t wifi_restart(void);

bool wifi_is_connected(void);

// Need to define functions as extern if compiling for C++
#ifdef __cplusplus
}
#endif

#endif