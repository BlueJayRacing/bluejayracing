#pragma once
#ifndef _LOCK_GUARD_HPP_
#define _LOCK_GUARD_HPP_

#include <esp_system.h>
#include <freertos/FreeRTOS.h>

class lockGuard {
    public:
        lockGuard(SemaphoreHandle_t t_mutex);
        ~lockGuard(void);
    private:
        SemaphoreHandle_t mutex_;
};

#endif