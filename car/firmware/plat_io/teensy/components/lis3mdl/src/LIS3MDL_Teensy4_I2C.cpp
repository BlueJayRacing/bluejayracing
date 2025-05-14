/*!
 * @file     LIS3MDL_Teensy4_I2C.cpp
 *
 * @mainpage Asynchronous LIS3MDL Library for Teensy 4
 *
 * @section intro_sec Introduction
 *
 * This is an asynchronous implementation of the LIS3MDL magnetometer library
 * optimized for Teensy 4.x using the teensy4_i2c library
 */

 #include "LIS3MDL_Teensy4_I2C.h"
 #include "Arduino.h"
 
 /**
  * @brief Construct a new LIS3MDL Teensy 4 I2C object
  */
 LIS3MDL_Teensy4_I2C::LIS3MDL_Teensy4_I2C() {
   _i2c_master = nullptr;
   _async_state = LIS3MDL_IDLE;
   _range = LIS3MDL_RANGE_4_GAUSS;
   _data_rate = LIS3MDL_DATARATE_155_HZ;
   _performance_mode = LIS3MDL_ULTRAHIGHMODE;
   _operation_mode = LIS3MDL_CONTINUOUSMODE;
   _sensorID = 12345; // Arbitrary ID
 }
 
 /**
  * @brief Destroy the LIS3MDL Teensy 4 I2C object
  */
 LIS3MDL_Teensy4_I2C::~LIS3MDL_Teensy4_I2C() {
   // We don't delete _i2c_master because we're using global instances
 }
 
 /**
  * @brief Initialize the LIS3MDL sensor
  * 
  * @param i2c_addr I2C address of the sensor (default: 0x1C)
  * @param bus Which I2C bus to use (default: BUS0)
  * @return true if initialization successful
  * @return false if initialization failed
  */
 bool LIS3MDL_Teensy4_I2C::begin(uint8_t i2c_addr, lis3mdl_i2c_bus_t bus) {
   _i2c_addr = i2c_addr;
   _i2c_bus = bus;
   
   // Select the appropriate I2C master based on the bus
   switch (bus) {
     case LIS3MDL_I2C_BUS0:
       _i2c_master = &Master;
       break;
     case LIS3MDL_I2C_BUS1:
       _i2c_master = &Master1;
       break;
     case LIS3MDL_I2C_BUS2:
       _i2c_master = &Master2;
       break;
     default:
       _i2c_master = &Master; // Default to Bus 0
       break;
   }
   
   // Initialize I2C with typical frequency for sensor communication (400kHz)
   _i2c_master->begin(400000);
   
   // Enable internal pullups for reliable communication
   _i2c_master->set_internal_pullups(InternalPullup::enabled_22k_ohm);
   
   // Allow time for I2C bus to stabilize
   delay(10);
   
   return _init();
 }
 
 /**
  * @brief Initialize the sensor and verify communication
  * 
  * @return true if initialization successful
  * @return false if initialization failed
  */
 bool LIS3MDL_Teensy4_I2C::_init() {
   uint8_t chip_id = 0;
   
   // Check if we can communicate with the device by reading WHO_AM_I register
   if (!readRegister(LIS3MDL_REG_WHO_AM_I, &chip_id)) {
     return false;
   }
   
   // Verify device ID (should be 0x3D for LIS3MDL)
   if (chip_id != 0x3D) {
     return false;
   }
   
   // Perform a reset
   reset();
   
   // Wait a bit longer for the reset to complete
   delay(20);
   
   // Configure with default settings
   setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
   setDataRate(LIS3MDL_DATARATE_155_HZ);
   setRange(LIS3MDL_RANGE_4_GAUSS);
   setOperationMode(LIS3MDL_CONTINUOUSMODE);
   
   return true;
 }
 
 /**
  * @brief Reset the sensor to default settings
  */
 void LIS3MDL_Teensy4_I2C::reset() {
   // Set the REBOOT bit in CTRL_REG2
   writeRegister(LIS3MDL_REG_CTRL_REG2, 0x04);
   delay(10); // Wait for reboot to complete
   
   // Update cached range value
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG2, &reg_value);
   _range = (lis3mdl_range_t)((reg_value >> 5) & 0x03);
 }
 
 /**
  * @brief Set the performance mode for the sensor
  * 
  * @param mode Performance mode (Low Power, Medium, High, Ultra-high)
  */
 void LIS3MDL_Teensy4_I2C::setPerformanceMode(lis3mdl_performancemode_t mode) {
   // Read current CTRL_REG1 value
   uint8_t reg1_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG1, &reg1_value);
   
   // Update performance mode bits (bits 5-6)
   reg1_value &= ~(0x60); // Clear bits 5-6
   reg1_value |= (((uint8_t)mode) << 5); // Set new performance mode
   
   // Write back to CTRL_REG1
   writeRegister(LIS3MDL_REG_CTRL_REG1, reg1_value);
   
   // Do the same for Z-axis performance mode in CTRL_REG4
   uint8_t reg4_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG4, &reg4_value);
   
   reg4_value &= ~(0x0C); // Clear bits 2-3
   reg4_value |= (((uint8_t)mode) << 2); // Set new performance mode
   
   writeRegister(LIS3MDL_REG_CTRL_REG4, reg4_value);
   
   _performance_mode = mode;
 }
 
 /**
  * @brief Get the current performance mode
  * 
  * @return lis3mdl_performancemode_t Current performance mode
  */
 lis3mdl_performancemode_t LIS3MDL_Teensy4_I2C::getPerformanceMode() {
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG1, &reg_value);
   return (lis3mdl_performancemode_t)((reg_value >> 5) & 0x03);
 }
 
 /**
  * @brief Set the operation mode (continuous, single, power-down)
  * 
  * @param mode Operation mode
  */
 void LIS3MDL_Teensy4_I2C::setOperationMode(lis3mdl_operationmode_t mode) {
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG3, &reg_value);
   
   reg_value &= ~(0x03); // Clear bits 0-1
   reg_value |= (uint8_t)mode; // Set new operation mode
   
   writeRegister(LIS3MDL_REG_CTRL_REG3, reg_value);
   
   _operation_mode = mode;
 }
 
 /**
  * @brief Get the current operation mode
  * 
  * @return lis3mdl_operationmode_t Current operation mode
  */
 lis3mdl_operationmode_t LIS3MDL_Teensy4_I2C::getOperationMode() {
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG3, &reg_value);
   return (lis3mdl_operationmode_t)(reg_value & 0x03);
 }
 
 /**
  * @brief Set the data rate for the sensor
  * 
  * @param dataRate Data rate enum value
  */
 void LIS3MDL_Teensy4_I2C::setDataRate(lis3mdl_dataRate_t dataRate) {
   // Set appropriate performance mode based on data rate
   if (dataRate == LIS3MDL_DATARATE_155_HZ) {
     setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
   } else if (dataRate == LIS3MDL_DATARATE_300_HZ) {
     setPerformanceMode(LIS3MDL_HIGHMODE);
   } else if (dataRate == LIS3MDL_DATARATE_560_HZ) {
     setPerformanceMode(LIS3MDL_MEDIUMMODE);
   } else if (dataRate == LIS3MDL_DATARATE_1000_HZ) {
     setPerformanceMode(LIS3MDL_LOWPOWERMODE);
   }
   
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG1, &reg_value);
   
   // Clear bits 1-4 (data rate bits)
   reg_value &= ~(0x1E);
   
   // Set new data rate
   reg_value |= (((uint8_t)dataRate) << 1);
   
   writeRegister(LIS3MDL_REG_CTRL_REG1, reg_value);
   
   _data_rate = dataRate;
 }
 
 /**
  * @brief Get the current data rate
  * 
  * @return lis3mdl_dataRate_t Current data rate
  */
 lis3mdl_dataRate_t LIS3MDL_Teensy4_I2C::getDataRate() {
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG1, &reg_value);
   return (lis3mdl_dataRate_t)((reg_value >> 1) & 0x0F);
 }
 
 /**
  * @brief Set the measurement range
  * 
  * @param range Measurement range (4, 8, 12, or 16 gauss)
  */
 void LIS3MDL_Teensy4_I2C::setRange(lis3mdl_range_t range) {
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG2, &reg_value);
   
   // Clear bits 5-6 (range bits)
   reg_value &= ~(0x60);
   
   // Set new range
   reg_value |= (((uint8_t)range) << 5);
   
   writeRegister(LIS3MDL_REG_CTRL_REG2, reg_value);
   
   _range = range;
 }
 
 /**
  * @brief Get the current measurement range
  * 
  * @return lis3mdl_range_t Current range
  */
 lis3mdl_range_t LIS3MDL_Teensy4_I2C::getRange() {
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG2, &reg_value);
   return (lis3mdl_range_t)((reg_value >> 5) & 0x03);
 }
 
 /**
  * @brief Set the interrupt threshold value
  * 
  * @param value Threshold value (16-bit)
  */
 void LIS3MDL_Teensy4_I2C::setIntThreshold(uint16_t value) {
   value &= 0x7FFF; // Make sure high bit is 0
   
   // Write low byte
   writeRegister(LIS3MDL_REG_INT_THS_L, value & 0xFF);
   
   // Write high byte
   writeRegister(LIS3MDL_REG_INT_THS_L + 1, (value >> 8) & 0xFF);
 }
 
 /**
  * @brief Get the current interrupt threshold value
  * 
  * @return uint16_t Current threshold value
  */
 uint16_t LIS3MDL_Teensy4_I2C::getIntThreshold() {
   uint8_t low_byte = 0, high_byte = 0;
   
   readRegister(LIS3MDL_REG_INT_THS_L, &low_byte);
   readRegister(LIS3MDL_REG_INT_THS_L + 1, &high_byte);
   
   return ((uint16_t)high_byte << 8) | low_byte;
 }
 
 /**
  * @brief Configure interrupt settings
  * 
  * @param enableX Enable interrupt on X axis
  * @param enableY Enable interrupt on Y axis
  * @param enableZ Enable interrupt on Z axis
  * @param polarity Interrupt pin polarity
  * @param latch Latch interrupt
  * @param enableInt Enable interrupt output
  */
 void LIS3MDL_Teensy4_I2C::configInterrupt(bool enableX, bool enableY, bool enableZ, 
                                         bool polarity, bool latch, bool enableInt) {
   uint8_t value = 0x08; // Default bit 3 is set
   
   if (enableX) value |= (1 << 7);
   if (enableY) value |= (1 << 6);
   if (enableZ) value |= (1 << 5);
   if (polarity) value |= (1 << 2);
   if (latch) value |= (1 << 1);
   if (enableInt) value |= 1;
   
   writeRegister(LIS3MDL_REG_INT_CFG, value);
 }
 
 /**
  * @brief Enable or disable self-test
  * 
  * @param flag True to enable self-test, false to disable
  */
 void LIS3MDL_Teensy4_I2C::selfTest(bool flag) {
   uint8_t reg_value = 0;
   readRegister(LIS3MDL_REG_CTRL_REG1, &reg_value);
   
   if (flag) {
     reg_value |= 0x01; // Set bit 0 (ST)
   } else {
     reg_value &= ~0x01; // Clear bit 0 (ST)
   }
   
   writeRegister(LIS3MDL_REG_CTRL_REG1, reg_value);
 }
 
 /**
 * Fixed Read Implementation for LIS3MDL_Teensy4_I2C
 * 
 * This is a focused fix for the read functions that weren't working properly.
 * Replace these functions in your LIS3MDL_Teensy4_I2C.cpp file.
 */

/**
 * @brief Start an asynchronous read of the sensor data
 * 
 * This function initiates a read of the XYZ data registers but returns
 * immediately. Call isReadComplete() to check when the data is available.
 */
void LIS3MDL_Teensy4_I2C::readAsync() {
  // Don't start a new read if one is already in progress
  if (_async_state != LIS3MDL_IDLE) {
    return;
  }
  
  // Ensure we have a valid I2C master
  if (_i2c_master == nullptr) {
    return;
  }
  
  // Start reading from OUT_X_L register (0x28) with auto-increment
  _buffer[0] = LIS3MDL_REG_OUT_X_L | 0x80;  // Add auto-increment bit (0x80)
  
  // Debug: print the register address we're sending
  Serial.print("Read reg addr: 0x");
  Serial.println(_buffer[0], HEX);
  
  _i2c_master->write_async(_i2c_addr, _buffer, 1, false);
  _async_state = LIS3MDL_WRITING_REG;
}
 
 /**
 * @brief Check if asynchronous read is complete
 * 
 * @return true if read is complete and data is available
 * @return false if read is still in progress
 */
bool LIS3MDL_Teensy4_I2C::isReadComplete() {
  // If we're idle, we're done
  if (_async_state == LIS3MDL_IDLE) {
    return true;
  }
  
  // Make sure we have a valid I2C master
  if (_i2c_master == nullptr) {
    Serial.println("I2C master is null");
    _async_state = LIS3MDL_IDLE;
    return true;
  }
  
  // Check if current operation is finished
  if (!_i2c_master->finished()) {
    return false;
  }
  
  // Check for I2C errors
  if (_i2c_master->error() != I2CError::ok) {
    Serial.print("I2C error: ");
    Serial.println((int)_i2c_master->error());
    // Reset state on error
    _async_state = LIS3MDL_IDLE;
    return true;
  }
  
  // State machine to handle the async read process
  switch (_async_state) {
    case LIS3MDL_WRITING_REG:
      Serial.println("Address sent, reading data");
      // Register address sent, now read the data
      _i2c_master->read_async(_i2c_addr, _buffer, 6, true);
      _async_state = LIS3MDL_READING_DATA;
      return false;
      
    case LIS3MDL_READING_DATA:
      Serial.println("Data read complete, processing");
      // Data read complete, process it
      processReadData();
      _async_state = LIS3MDL_IDLE;
      return true;
      
    default:
      Serial.println("Unknown state");
      // Unknown state, reset
      _async_state = LIS3MDL_IDLE;
      return true;
  }
}
 
/**
 * @brief Process the read data from the sensor
 */
void LIS3MDL_Teensy4_I2C::processReadData() {
  // Extract X, Y, Z values from buffer
  x = _buffer[0];
  x |= _buffer[1] << 8;
  
  y = _buffer[2];
  y |= _buffer[3] << 8;
  
  z = _buffer[4];
  z |= _buffer[5] << 8;
  
  // Print raw values for debugging
  Serial.print("Raw data - X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(", Z: ");
  Serial.println(z);
  
  // Convert raw values to gauss based on current range setting
  float scale = getGaussPerLSB();
  
  x_gauss = (float)x / scale;
  y_gauss = (float)y / scale;
  z_gauss = (float)z / scale;
}
 
 /**
  * @brief Get the scale factor (LSB per gauss) for the current range
  * 
  * @return float Scale factor
  */
 float LIS3MDL_Teensy4_I2C::getGaussPerLSB() {
   switch (_range) {
     case LIS3MDL_RANGE_16_GAUSS:
       return 1711.0f;
     case LIS3MDL_RANGE_12_GAUSS:
       return 2281.0f;
     case LIS3MDL_RANGE_8_GAUSS:
       return 3421.0f;
     case LIS3MDL_RANGE_4_GAUSS:
     default:
       return 6842.0f;
   }
 }
 
 /**
 * @brief Synchronous read of sensor data
 * 
 * This function blocks until data is available
 */
void LIS3MDL_Teensy4_I2C::read() {
  // Start an asynchronous read
  readAsync();
  
  // Wait for read to complete with a timeout
  unsigned long startTime = millis();
  unsigned long timeout = 100; // 100ms timeout
  
  while (!isReadComplete()) {
    if (millis() - startTime > timeout) {
      Serial.println("Read timeout!");
      break;
    }
    delayMicroseconds(100);
  }
}
 
 /**
  * @brief Get a sensor event with the current data
  * 
  * @param event Pointer to event structure to fill
  * @return true if successful
  * @return false if failed
  */
 bool LIS3MDL_Teensy4_I2C::getEvent(sensors_event_t* event) {
   // Clear the event
   memset(event, 0, sizeof(sensors_event_t));
   
   event->version = sizeof(sensors_event_t);
   event->sensor_id = _sensorID;
   event->type = SENSOR_TYPE_MAGNETIC_FIELD;
   event->timestamp = millis();
   
   // Make sure we have fresh data
   read();
   
   // Convert to microtesla (1 gauss = 100 microtesla)
   event->magnetic.x = x_gauss * 100;
   event->magnetic.y = y_gauss * 100;
   event->magnetic.z = z_gauss * 100;
   
   return true;
 }
 
 /**
  * @brief Get sensor details
  * 
  * @param sensor Pointer to sensor_t structure to fill
  */
 void LIS3MDL_Teensy4_I2C::getSensor(sensor_t* sensor) {
   // Clear the sensor_t object
   memset(sensor, 0, sizeof(sensor_t));
   
   // Fill in details
   strncpy(sensor->name, "LIS3MDL", sizeof(sensor->name) - 1);
   sensor->name[sizeof(sensor->name) - 1] = 0;
   sensor->version = 1;
   sensor->sensor_id = _sensorID;
   sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
   sensor->min_delay = 0;
   sensor->min_value = -1600;  // -16 gauss in uTesla
   sensor->max_value = 1600;   // +16 gauss in uTesla
   sensor->resolution = 0.015; // 100/6842 uTesla per LSB at +-4 gauss range
 }
 
 /**
  * @brief Arduino compatible API for reading magnetic field data
  * 
  * @param x Reference to store X axis value in microtesla
  * @param y Reference to store Y axis value in microtesla
  * @param z Reference to store Z axis value in microtesla
  * @return int 1 if successful, 0 if failed
  */
 int LIS3MDL_Teensy4_I2C::readMagneticField(float &x, float &y, float &z) {
   read();
   
   x = x_gauss * 100; // Convert to microtesla
   y = y_gauss * 100;
   z = z_gauss * 100;
   
   return 1;
 }
 
 /**
  * @brief Get the sample rate in Hz
  * 
  * @return float Sample rate
  */
 float LIS3MDL_Teensy4_I2C::magneticFieldSampleRate() {
   switch (_data_rate) {
     case LIS3MDL_DATARATE_0_625_HZ:
       return 0.625f;
     case LIS3MDL_DATARATE_1_25_HZ:
       return 1.25f;
     case LIS3MDL_DATARATE_2_5_HZ:
       return 2.5f;
     case LIS3MDL_DATARATE_5_HZ:
       return 5.0f;
     case LIS3MDL_DATARATE_10_HZ:
       return 10.0f;
     case LIS3MDL_DATARATE_20_HZ:
       return 20.0f;
     case LIS3MDL_DATARATE_40_HZ:
       return 40.0f;
     case LIS3MDL_DATARATE_80_HZ:
       return 80.0f;
     case LIS3MDL_DATARATE_155_HZ:
       return 155.0f;
     case LIS3MDL_DATARATE_300_HZ:
       return 300.0f;
     case LIS3MDL_DATARATE_560_HZ:
       return 560.0f;
     case LIS3MDL_DATARATE_1000_HZ:
       return 1000.0f;
     default:
       return 0.0f;
   }
 }
 
 /**
  * @brief Check if new data is available
  * 
  * @return int 1 if data available, 0 if not
  */
 int LIS3MDL_Teensy4_I2C::magneticFieldAvailable() {
   uint8_t status = 0;
   readRegister(LIS3MDL_REG_STATUS, &status);
   return (status & 0x08) ? 1 : 0;
 }
 
 /**
  * @brief Write a value to a register
  * 
  * @param reg Register address
  * @param value Value to write
  * @return true if successful
  * @return false if failed
  */
 bool LIS3MDL_Teensy4_I2C::writeRegister(uint8_t reg, uint8_t value) {
   if (_i2c_master == nullptr) {
     return false;
   }
   
   _buffer[0] = reg;
   _buffer[1] = value;
   
   // Write register address and value
   _i2c_master->write_async(_i2c_addr, _buffer, 2, true);
   
   // Wait for write to complete with timeout
   unsigned long startTime = millis();
   unsigned long timeout = 50; // 50ms timeout
   
   while (!_i2c_master->finished()) {
     if (millis() - startTime > timeout) {
       return false;
     }
     delayMicroseconds(100);
   }
   
   return (_i2c_master->error() == I2CError::ok);
 }
 
 /**
 * @brief Read a value from a register
 * 
 * @param reg Register address
 * @param value Pointer to store the read value
 * @return true if successful
 * @return false if failed
 */
bool LIS3MDL_Teensy4_I2C::readRegister(uint8_t reg, uint8_t* value) {
  if (_i2c_master == nullptr || value == nullptr) {
    return false;
  }
  
  _buffer[0] = reg;
  
  // Write register address
  _i2c_master->write_async(_i2c_addr, _buffer, 1, false);
  
  // Wait for write to complete with timeout
  unsigned long startTime = millis();
  unsigned long timeout = 50; // 50ms timeout
  
  while (!_i2c_master->finished()) {
    if (millis() - startTime > timeout) {
      return false;
    }
    delayMicroseconds(100);
  }
  
  if (_i2c_master->error() != I2CError::ok) {
    Serial.print("Register write error: ");
    Serial.println((int)_i2c_master->error());
    return false;
  }
  
  // Read register value
  _i2c_master->read_async(_i2c_addr, _buffer, 1, true);
  
  // Wait for read to complete with timeout
  startTime = millis();
  
  while (!_i2c_master->finished()) {
    if (millis() - startTime > timeout) {
      return false;
    }
    delayMicroseconds(100);
  }
  
  if (_i2c_master->error() == I2CError::ok) {
    *value = _buffer[0];
    return true;
  }
  
  Serial.print("Register read error: ");
  Serial.println((int)_i2c_master->error());
  return false;
}