#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <config.hpp>
#include <mqttManager.hpp>
#include <ad5689r.hpp>
#include <ads1120.hpp>
#include <calSensorSetup.hpp>
#include <driveSensorSetup.hpp>

#if ENABLE_TESTS == 1

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testMQTTManager(void);
    void testADCDACEndtoEnd(void);
    void testADCDACReadAnalogFrontEnd(void); // Reads the analog front end forever after zeroing
    // void testCalSensorSetup(void); // Reads the analog front end forever after zeroing
    // void testDriveSensorSetup(void); // Reads the analog front end forever after zeroing
    void testProtobufEncode(void);

  private:
    void testMQTTManagerBasicParamErrors(void);
    void testMQTTManagerWiFiConnectDisconnect(void);
    void testMQTTManagerClientConnectDisconnect(void);
    void testMQTTManagerClientWiFiConnectDisconnect(void);
    void testMQTTManagerMultipleClientsConDisCon(void);
    void testMQTTManagerClientPublishSubscribe(void);

    void testADCDACCheckSPIBus(void);
    void testADCDACReadDACBias(void);
    // void testADCDACTestADCGain2(void);
    // void testADCDACTestADCGain4(void);

    // void testCalSensorSetupReadAnalogFrontEnd(void);
    // void testCalSensorSetupZero(void);

    // void testDriveSensorSetupSPS(void);
    // void testDriveSensorSetupReadAnalogFrontEnd(void);
    // void testDriveSensorSetupZero(void);

    void testProtobufStockEncode(void);
    void testProtobufHardEncode(void);


  private:
    mqttManager* mqtt_manager_;
    AD5689R dac_;
    ADS1120 adc1_;
    ADS1120 adc2_;
    calSensorSetup cal_setup_;
    driveSensorSetup drive_setup_;
};

#endif

#endif