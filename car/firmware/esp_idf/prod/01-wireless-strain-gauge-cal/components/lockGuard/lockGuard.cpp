#include <lockGuard.hpp>

lockGuard::lockGuard(SemaphoreHandle_t t_mutex) : mutex_(t_mutex)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
}

lockGuard::~lockGuard(void) 
{
    xSemaphoreGive(mutex_);
}
