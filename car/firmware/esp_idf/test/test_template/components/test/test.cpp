#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>

static const char* TAG = "test";

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testClass(void) { testClassCase(); }

void testClassCase(void)
{
    ESP_LOGI(TAG, "Testing a case of the class");
    ESP_LOGI(TAG, "Passed testing a case of the class");
}