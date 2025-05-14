/*!
 * @file     LIS3MDL_Teensy4_I2C.h
 *
 * @mainpage Asynchronous LIS3MDL Library for Teensy 4
 *
 * @section intro_sec Introduction
 *
 * This is an asynchronous implementation of the LIS3MDL magnetometer library
 * optimized for Teensy 4.x using the teensy4_i2c library
 */

 #ifndef LIS3MDL_TEENSY4_I2C_H
 #define LIS3MDL_TEENSY4_I2C_H
 
 #include <Arduino.h>
 #include <Adafruit_Sensor.h>
 #include <i2c_driver.h>
 #include "imx_rt1060/imx_rt1060_i2c_driver.h"
 
 // LIS3MDL Register addresses and constants
 #define LIS3MDL_I2CADDR_DEFAULT (0x1C) ///< Default I2C address
 #define LIS3MDL_REG_WHO_AM_I 0x0F      ///< Register that contains the part ID
 #define LIS3MDL_REG_CTRL_REG1 0x20     ///< Register address for control 1
 #define LIS3MDL_REG_CTRL_REG2 0x21     ///< Register address for control 2
 #define LIS3MDL_REG_CTRL_REG3 0x22     ///< Register address for control 3
 #define LIS3MDL_REG_CTRL_REG4 0x23     ///< Register address for control 3
 #define LIS3MDL_REG_STATUS 0x27        ///< Register address for status
 #define LIS3MDL_REG_OUT_X_L 0x28       ///< Register address for X axis lower byte
 #define LIS3MDL_REG_INT_CFG 0x30       ///< Interrupt configuration register
 #define LIS3MDL_REG_INT_THS_L 0x32     ///< Low byte of the irq threshold
 
 /** The magnetometer ranges */
 typedef enum {
   LIS3MDL_RANGE_4_GAUSS = 0b00,  ///< +/- 4g (default value)
   LIS3MDL_RANGE_8_GAUSS = 0b01,  ///< +/- 8g
   LIS3MDL_RANGE_12_GAUSS = 0b10, ///< +/- 12g
   LIS3MDL_RANGE_16_GAUSS = 0b11, ///< +/- 16g
 } lis3mdl_range_t;
 
 /** The magnetometer data rate, includes FAST_ODR bit */
 typedef enum {
   LIS3MDL_DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
   LIS3MDL_DATARATE_1_25_HZ = 0b0010,  ///<  1.25 Hz
   LIS3MDL_DATARATE_2_5_HZ = 0b0100,   ///<  2.5 Hz
   LIS3MDL_DATARATE_5_HZ = 0b0110,     ///<  5 Hz
   LIS3MDL_DATARATE_10_HZ = 0b1000,    ///<  10 Hz
   LIS3MDL_DATARATE_20_HZ = 0b1010,    ///<  20 Hz
   LIS3MDL_DATARATE_40_HZ = 0b1100,    ///<  40 Hz
   LIS3MDL_DATARATE_80_HZ = 0b1110,    ///<  80 Hz
   LIS3MDL_DATARATE_155_HZ = 0b0001,   ///<  155 Hz (FAST_ODR + UHP)
   LIS3MDL_DATARATE_300_HZ = 0b0011,   ///<  300 Hz (FAST_ODR + HP)
   LIS3MDL_DATARATE_560_HZ = 0b0101,   ///<  560 Hz (FAST_ODR + MP)
   LIS3MDL_DATARATE_1000_HZ = 0b0111,  ///<  1000 Hz (FAST_ODR + LP)
 } lis3mdl_dataRate_t;
 
 /** The magnetometer performance mode */
 typedef enum {
   LIS3MDL_LOWPOWERMODE = 0b00,  ///< Low power mode
   LIS3MDL_MEDIUMMODE = 0b01,    ///< Medium performance mode
   LIS3MDL_HIGHMODE = 0b10,      ///< High performance mode
   LIS3MDL_ULTRAHIGHMODE = 0b11, ///< Ultra-high performance mode
 } lis3mdl_performancemode_t;
 
 /** The magnetometer operation mode */
 typedef enum {
   LIS3MDL_CONTINUOUSMODE = 0b00, ///< Continuous conversion
   LIS3MDL_SINGLEMODE = 0b01,     ///< Single-shot conversion
   LIS3MDL_POWERDOWNMODE = 0b11,  ///< Powered-down mode
 } lis3mdl_operationmode_t;
 
 /** Asynchronous operation states */
 typedef enum {
   LIS3MDL_IDLE,            ///< No operation in progress
   LIS3MDL_WRITING_REG,     ///< Writing to a register
   LIS3MDL_READING_REG,     ///< Reading from a register
   LIS3MDL_READING_DATA,    ///< Reading sensor data
 } lis3mdl_async_state_t;
 
 /** State machine for read operations from main sketch */
 typedef enum {
   IDLE,              ///< No operation in progress
   READING_MAG1_ADDR, ///< Sending register address to mag1
   READING_MAG1_DATA, ///< Reading data from mag1
   READING_MAG2_ADDR, ///< Sending register address to mag2
   READING_MAG2_DATA, ///< Reading data from mag2
   COMPLETE           ///< Read cycle complete
 } ReadState;
 
 /**
  * @brief Enumeration to identify which I2C bus to use
  */
 typedef enum {
   LIS3MDL_I2C_BUS0 = 0,  ///< I2C Bus 0 (SCL0 - pin 19, SDA0 - pin 18)
   LIS3MDL_I2C_BUS1 = 1,  ///< I2C Bus 1 (SCL1 - pin 16, SDA1 - pin 17)
   LIS3MDL_I2C_BUS2 = 2,  ///< I2C Bus 2 (SCL2 - pin 24, SDA2 - pin 25)
 } lis3mdl_i2c_bus_t;
 
 /**
  * @brief Asynchronous LIS3MDL class adapted for Teensy 4 I2C
  * 
  * This class provides asynchronous non-blocking I2C operations for the LIS3MDL 
  * magnetometer using the teensy4_i2c library.
  */
 class LIS3MDL_Teensy4_I2C : public Adafruit_Sensor {
 public:
   LIS3MDL_Teensy4_I2C();
   ~LIS3MDL_Teensy4_I2C();
 
   // Initialization
   bool begin(uint8_t i2c_addr = LIS3MDL_I2CADDR_DEFAULT, lis3mdl_i2c_bus_t bus = LIS3MDL_I2C_BUS0);
   void reset();
 
   // Configuration methods
   void setPerformanceMode(lis3mdl_performancemode_t mode);
   lis3mdl_performancemode_t getPerformanceMode();
   void setOperationMode(lis3mdl_operationmode_t mode);
   lis3mdl_operationmode_t getOperationMode();
   void setDataRate(lis3mdl_dataRate_t dataRate);
   lis3mdl_dataRate_t getDataRate();
   void setRange(lis3mdl_range_t range);
   lis3mdl_range_t getRange();
   void setIntThreshold(uint16_t value);
   uint16_t getIntThreshold();
   void configInterrupt(bool enableX, bool enableY, bool enableZ, bool polarity,
                       bool latch, bool enableInt);
   void selfTest(bool flag);
 
   // Asynchronous read operations
   void readAsync();
   bool isReadComplete();
   
   // Synchronous read operations (for compatibility)
   void read();
   bool getEvent(sensors_event_t* event);
   void getSensor(sensor_t* sensor);
 
   // Arduino compatible API
   int readMagneticField(float &x, float &y, float &z);
   float magneticFieldSampleRate();
   int magneticFieldAvailable();
 
   // Raw and processed data
   int16_t x,     ///< The last read X mag in raw units
       y,         ///< The last read Y mag in raw units
       z;         ///< The last read Z mag in raw units
   float x_gauss, ///< The last read X mag in 'gauss'
       y_gauss,   ///< The last read Y mag in 'gauss'
       z_gauss;   ///< The last read Z mag in 'gauss'
 
 private:
   bool _init();
   bool writeRegister(uint8_t reg, uint8_t value);
   bool readRegister(uint8_t reg, uint8_t* value);
   void processReadData();
   float getGaussPerLSB();
 
   // I2C communication
   I2CMaster* _i2c_master;  // Reference to I2C master
   uint8_t _i2c_addr;       // Device I2C address
   lis3mdl_i2c_bus_t _i2c_bus; // Which I2C bus to use
   
   // Asynchronous state tracking
   lis3mdl_async_state_t _async_state;
   uint8_t _buffer[6];  // Buffer for read/write operations
   
   // Sensor configuration
   lis3mdl_range_t _range;
   lis3mdl_dataRate_t _data_rate;
   lis3mdl_performancemode_t _performance_mode;
   lis3mdl_operationmode_t _operation_mode;
   
   int32_t _sensorID;
 };
 
 #endif // LIS3MDL_TEENSY4_I2C_H