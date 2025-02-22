#include <config.hpp>
#include <unity.h>

#include "verify_file_writer.hpp"

fileWriter* file_writer_ = new fileWriter();

void test_file_writer_error_handling(void) {
    access_channel_config_t invalid_cfg_1 = {"", ANALOG, .analog = {STRAIN, AIN0, REF_M}, 40};
    access_channel_config_t invalid_cfg_2 = {"left_axle", ANALOG, .analog = {STRAIN, AIN0, REF_M}, 0};

    // Fails on passing in invalid channel paramters
    TEST_ASSERT_EQUAL(file_writer_->addChannel(invalid_cfg_1), -1);
    TEST_ASSERT_EQUAL(file_writer_->addChannel(invalid_cfg_2), -1);

    // Fails on passing in invalid channel index
    TEST_ASSERT_EQUAL(file_writer_->logChannel(1, 0xFFFF), -1);
    TEST_ASSERT_EQUAL(file_writer_->getBufferSize(1), -1);
    TEST_ASSERT_EQUAL(file_writer_->getDataSize(1), -1);
}

void test_file_writer_one_channel(void) {

}

void test_file_writer_mult_channel(void) {

}
