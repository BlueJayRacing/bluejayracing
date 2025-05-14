/*
* Dual LIS3MDL Magnetometers Example with Optimized I2C
* 
* This example demonstrates how to use two LIS3MDL magnetometers
* with the LIS3MDL_Teensy4_I2C library. This is a more advanced
* example that uses direct I2C commands for timing optimization.
*/

#include <LIS3MDL_Teensy4_I2C.h>

// Sensor addresses
#define MAG1_ADDR 0x1C
#define MAG2_ADDR 0x1E

// I2C Clock frequency - pushing to maximum for faster transfers
#define I2C_FREQUENCY 1000000  // 1 MHz (Fast Mode Plus)

// Timing variables
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 100; // Read every 100ms
unsigned long cycle_start_time = 0;
unsigned long cycle_total_time = 0;

// Timing variables - magnetometer 1
unsigned long mag1_start_time = 0;
unsigned long mag1_elapsed = 0;
unsigned long mag1_addr_start = 0;
unsigned long mag1_addr_time = 0;
unsigned long mag1_data_start = 0;
unsigned long mag1_data_time = 0;
unsigned long mag1_process_start = 0;
unsigned long mag1_process_time = 0;

// Timing variables - magnetometer 2
unsigned long mag2_start_time = 0;
unsigned long mag2_elapsed = 0;
unsigned long mag2_addr_start = 0;
unsigned long mag2_addr_time = 0;
unsigned long mag2_data_start = 0;
unsigned long mag2_data_time = 0;
unsigned long mag2_process_start = 0;
unsigned long mag2_process_time = 0;

// Data storage
int16_t mag1_x = 0, mag1_y = 0, mag1_z = 0;
int16_t mag2_x = 0, mag2_y = 0, mag2_z = 0;
float mag1_x_gauss = 0, mag1_y_gauss = 0, mag1_z_gauss = 0;
float mag2_x_gauss = 0, mag2_y_gauss = 0, mag2_z_gauss = 0;

// Buffer for reading data
uint8_t read_buffer[6];
uint8_t reg_addr;

// State machine for read operations
ReadState currentState = IDLE;

// Debug flag - set to true for verbose output
const bool DEBUG = true;

// Current sensor configuration (for reference)
lis3mdl_performancemode_t current_perf_mode = LIS3MDL_ULTRAHIGHMODE;
lis3mdl_operationmode_t current_op_mode = LIS3MDL_CONTINUOUSMODE;
lis3mdl_dataRate_t current_data_rate = LIS3MDL_DATARATE_155_HZ;
lis3mdl_range_t current_range = LIS3MDL_RANGE_4_GAUSS;



bool initMagnetometer(uint8_t address);
void printConfiguration();
void startReadCycle();
void startMag2Read();
void processMag1Data();
void processMag2Data();
void printTimingData();
void printSensorData();
bool readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t* value);
bool writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value);


void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for serial port to open
  while (!Serial) delay(10);
  
  Serial.println("\nLIS3MDL Dual Magnetometer with Optimized I2C");
  Serial.println("=============================================");
  
  // Initialize I2C bus with maximum speed for fastest possible transfers
  Master.begin(I2C_FREQUENCY);
  
  // Enable internal pullups for reliable communication
  Master.set_internal_pullups(InternalPullup::enabled_22k_ohm);
  
  // Set pad control configuration for fastest possible edges
  // This is an advanced setting that can improve I2C speed
  Master.set_pad_control_configuration(0x1F830);
  
  // Wait for I2C bus to stabilize
  delay(10);
  
  // Initialize both magnetometers
  if (!initMagnetometer(MAG1_ADDR)) {
    Serial.println("ERROR: Could not initialize magnetometer 1!");
    while (1) delay(10);
  }
  
  if (!initMagnetometer(MAG2_ADDR)) {
    Serial.println("ERROR: Could not initialize magnetometer 2!");
    while (1) delay(10);
  }
  
  Serial.println("Both magnetometers initialized successfully!");
  
  // Print the current configuration
  printConfiguration();
  
  // Print the header for our data output
  if (DEBUG) {
    Serial.println("--- Detailed Timing Information (all times in microseconds) ---");
  } else {
    Serial.println("M1_X,M1_Y,M1_Z,M1_Time,M2_X,M2_Y,M2_Z,M2_Time,Total_Time");
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  // State machine for the read process
  switch (currentState) {
    case IDLE:
      // Start a new read cycle if it's time
      if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;
        cycle_start_time = micros();  // Start timing the whole cycle
        startReadCycle();
      }
      break;
      
    case READING_MAG1_ADDR:
      // Check if address write for mag1 is complete
      if (Master.finished()) {
        mag1_addr_time = micros() - mag1_addr_start;
        
        if (Master.error() == I2CError::ok) {
          // Start reading data from mag1
          if (DEBUG) Serial.println("Reading data from mag1...");
          mag1_data_start = micros();
          Master.read_async(MAG1_ADDR, read_buffer, 6, true);
          currentState = READING_MAG1_DATA;
        } else {
          if (DEBUG) {
            Serial.print("Error writing mag1 address: ");
            Serial.println((int)Master.error());
          }
          currentState = IDLE;
        }
      }
      break;
      
    case READING_MAG1_DATA:
      // Check if data read for mag1 is complete
      if (Master.finished()) {
        mag1_data_time = micros() - mag1_data_start;
        
        if (Master.error() == I2CError::ok) {
          // Process mag1 data
          mag1_process_start = micros();
          processMag1Data();
          mag1_process_time = micros() - mag1_process_start;
          
          // Now start mag2 read
          startMag2Read();
        } else {
          if (DEBUG) {
            Serial.print("Error reading mag1 data: ");
            Serial.println((int)Master.error());
          }
          currentState = IDLE;
        }
      }
      break;
      
    case READING_MAG2_ADDR:
      // Check if address write for mag2 is complete
      if (Master.finished()) {
        mag2_addr_time = micros() - mag2_addr_start;
        
        if (Master.error() == I2CError::ok) {
          // Start reading data from mag2
          if (DEBUG) Serial.println("Reading data from mag2...");
          mag2_data_start = micros();
          Master.read_async(MAG2_ADDR, read_buffer, 6, true);
          currentState = READING_MAG2_DATA;
        } else {
          if (DEBUG) {
            Serial.print("Error writing mag2 address: ");
            Serial.println((int)Master.error());
          }
          currentState = IDLE;
        }
      }
      break;
      
    case READING_MAG2_DATA:
      // Check if data read for mag2 is complete
      if (Master.finished()) {
        mag2_data_time = micros() - mag2_data_start;
        
        if (Master.error() == I2CError::ok) {
          // Process mag2 data
          mag2_process_start = micros();
          processMag2Data();
          mag2_process_time = micros() - mag2_process_start;
          
          // Calculate total cycle time
          cycle_total_time = micros() - cycle_start_time;
          
          // Read cycle complete
          currentState = COMPLETE;
        } else {
          if (DEBUG) {
            Serial.print("Error reading mag2 data: ");
            Serial.println((int)Master.error());
          }
          currentState = IDLE;
        }
      }
      break;
      
    case COMPLETE:
      // Print the results
      printTimingData();
      printSensorData();
      
      // Reset for next cycle
      currentState = IDLE;
      break;
  }
}

// Initialize a magnetometer with optimal settings for speed
bool initMagnetometer(uint8_t address) {
  Serial.print("Initializing magnetometer at address 0x");
  Serial.println(address, HEX);
  
  // Check WHO_AM_I register
  uint8_t whoami = 0;
  if (!readRegister(address, LIS3MDL_REG_WHO_AM_I, &whoami)) {
    Serial.println("Failed to read WHO_AM_I register");
    return false;
  }
  
  if (whoami != 0x3D) {
    Serial.print("Invalid WHO_AM_I value: 0x");
    Serial.println(whoami, HEX);
    return false;
  }
  
  Serial.println("WHO_AM_I check passed");
  
  // Reset the device
  writeRegister(address, LIS3MDL_REG_CTRL_REG2, 0x04);
  delay(20); // Wait for reset to complete
  
  // Configure for maximum speed and performance
  
  // CTRL_REG1: Ultra-high performance mode, Fast ODR, 1000 Hz data rate
  // 0x82: OM = 11 (ultra-high perf), DO = 0, FAST_ODR = 1, ST = 0
  writeRegister(address, LIS3MDL_REG_CTRL_REG1, 0x82);
  
  // CTRL_REG2: ±4 gauss full scale
  // 0x00: FS = 00 (±4 gauss)
  writeRegister(address, LIS3MDL_REG_CTRL_REG2, 0x00);
  
  // CTRL_REG3: Continuous conversion mode, normal I2C mode
  // 0x00: MD = 00 (continuous conversion)
  writeRegister(address, LIS3MDL_REG_CTRL_REG3, 0x00);
  
  // CTRL_REG4: Ultra-high performance mode for Z-axis
  // 0x0C: OMZ = 11 (ultra-high perf)
  writeRegister(address, LIS3MDL_REG_CTRL_REG4, 0x0C);
  
  // Update our tracking variables
  current_perf_mode = LIS3MDL_ULTRAHIGHMODE;
  current_op_mode = LIS3MDL_CONTINUOUSMODE;
  current_data_rate = LIS3MDL_DATARATE_1000_HZ;
  current_range = LIS3MDL_RANGE_4_GAUSS;
  
  Serial.println("Configuration complete");
  
  return true;
}

// Print current sensor configuration
void printConfiguration() {
  Serial.println("\nCurrent Magnetometer Configuration:");
  
  Serial.print("Performance Mode: ");
  switch(current_perf_mode) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low Power"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-high"); break;
  }
  
  Serial.print("Operation Mode: ");
  switch(current_op_mode) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }
  
  Serial.print("Data Rate: ");
  switch(current_data_rate) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
    default: Serial.println("Unknown"); break;
  }
  
  Serial.print("Range: +/- ");
  switch(current_range) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("16 gauss"); break;
  }
  
  Serial.print("I2C Bus Frequency: ");
  Serial.print(I2C_FREQUENCY / 1000);
  Serial.println(" kHz");
  
  Serial.println();
}


// Start a read cycle
void startReadCycle() {
  if (DEBUG) Serial.println("\nStarting read cycle...");
  
  // Start timing for mag1
  mag1_start_time = micros();
  
  // Start reading from mag1
  if (DEBUG) Serial.println("Starting read from mag1...");
  reg_addr = LIS3MDL_REG_OUT_X_L | 0x80;  // Use auto-increment
  mag1_addr_start = micros();
  Master.write_async(MAG1_ADDR, &reg_addr, 1, false);
  
  currentState = READING_MAG1_ADDR;
}


// Start reading from mag2
void startMag2Read() {
  // Start timing for mag2
  mag2_start_time = micros();
  
  // Start reading from mag2
  if (DEBUG) Serial.println("Starting read from mag2...");
  reg_addr = LIS3MDL_REG_OUT_X_L | 0x80;  // Use auto-increment
  mag2_addr_start = micros();
  Master.write_async(MAG2_ADDR, &reg_addr, 1, false);
  
  currentState = READING_MAG2_ADDR;
}

// Process the data from mag1
void processMag1Data() {
  // Calculate elapsed time
  mag1_elapsed = micros() - mag1_start_time;
  
  // Extract X, Y, Z values
  mag1_x = read_buffer[0] | (read_buffer[1] << 8);
  mag1_y = read_buffer[2] | (read_buffer[3] << 8);
  mag1_z = read_buffer[4] | (read_buffer[5] << 8);
  
  // Convert to gauss
  float gauss_per_lsb = 6842.0; // For ±4 gauss range
  mag1_x_gauss = (float)mag1_x / gauss_per_lsb;
  mag1_y_gauss = (float)mag1_y / gauss_per_lsb;
  mag1_z_gauss = (float)mag1_z / gauss_per_lsb;
  
  if (DEBUG) {
    Serial.print("Mag1 data: X=");
    Serial.print(mag1_x);
    Serial.print(", Y=");
    Serial.print(mag1_y);
    Serial.print(", Z=");
    Serial.println(mag1_z);
  }
}

// Process the data from mag2
void processMag2Data() {
  // Calculate elapsed time
  mag2_elapsed = micros() - mag2_start_time;
  
  // Extract X, Y, Z values
  mag2_x = read_buffer[0] | (read_buffer[1] << 8);
  mag2_y = read_buffer[2] | (read_buffer[3] << 8);
  mag2_z = read_buffer[4] | (read_buffer[5] << 8);
  
  // Convert to gauss
  float gauss_per_lsb = 6842.0; // For ±4 gauss range
  mag2_x_gauss = (float)mag2_x / gauss_per_lsb;
  mag2_y_gauss = (float)mag2_y / gauss_per_lsb;
  mag2_z_gauss = (float)mag2_z / gauss_per_lsb;
  
  if (DEBUG) {
    Serial.print("Mag2 data: X=");
    Serial.print(mag2_x);
    Serial.print(", Y=");
    Serial.print(mag2_y);
    Serial.print(", Z=");
    Serial.println(mag2_z);
  }
}

// Print detailed timing data
void printTimingData() {
  if (DEBUG) {
    Serial.println("\n--- Timing Results (microseconds) ---");
    
    Serial.print("Mag1 - Address Write: ");
    Serial.print(mag1_addr_time);
    Serial.print(", Data Read: ");
    Serial.print(mag1_data_time);
    Serial.print(", Processing: ");
    Serial.print(mag1_process_time);
    Serial.print(", Total: ");
    Serial.println(mag1_elapsed);
    
    Serial.print("Mag2 - Address Write: ");
    Serial.print(mag2_addr_time);
    Serial.print(", Data Read: ");
    Serial.print(mag2_data_time);
    Serial.print(", Processing: ");
    Serial.print(mag2_process_time);
    Serial.print(", Total: ");
    Serial.println(mag2_elapsed);
    
    Serial.print("Total Cycle Time: ");
    Serial.print(cycle_total_time);
    Serial.println(" microseconds");
    
    Serial.println("--------------------------------");
  }
}

// Print the sensor data in CSV format
void printSensorData() {
  if (!DEBUG) {
    // Print in CSV format with all timing data
    Serial.print(mag1_x);
    Serial.print(",");
    Serial.print(mag1_y);
    Serial.print(",");
    Serial.print(mag1_z);
    Serial.print(",");
    Serial.print(mag1_elapsed);
    Serial.print(",");
    Serial.print(mag2_x);
    Serial.print(",");
    Serial.print(mag2_y);
    Serial.print(",");
    Serial.print(mag2_z);
    Serial.print(",");
    Serial.print(mag2_elapsed);
    Serial.print(",");
    Serial.println(cycle_total_time);
  } else {
    // In debug mode, just print a simpler data line
    Serial.print("Data: M1(");
    Serial.print(mag1_x);
    Serial.print(",");
    Serial.print(mag1_y);
    Serial.print(",");
    Serial.print(mag1_z);
    Serial.print(") M2(");
    Serial.print(mag2_x);
    Serial.print(",");
    Serial.print(mag2_y);
    Serial.print(",");
    Serial.print(mag2_z);
    Serial.println(")");
  }
}

// Fast version of register reading
bool readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t* value) {
  // Write the register address
  Master.write_async(deviceAddr, &reg, 1, false);
  
  // Wait for completion
  while (!Master.finished()) {
    delayMicroseconds(10); // Reduced delay for faster polling
  }
  
  // Check for errors
  if (Master.error() != I2CError::ok) {
    return false;
  }
  
  // Read the register value
  Master.read_async(deviceAddr, value, 1, true);
  
  // Wait for completion
  while (!Master.finished()) {
    delayMicroseconds(10); // Reduced delay for faster polling
  }
  
  // Check for errors
  return (Master.error() == I2CError::ok);
}

// Fast version of register writing
bool writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  
  // Write the register address and value
  Master.write_async(deviceAddr, buffer, 2, true);
  
  // Wait for completion
  while (!Master.finished()) {
    delayMicroseconds(10); // Reduced delay for faster polling
  }
  
  // Check for errors
  return (Master.error() == I2CError::ok);
}