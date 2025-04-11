/*
 * Basic LIS3MDL Magnetometer Example
 * 
 * This example shows how to use the LIS3MDL_Teensy4_I2C library
 * to read data from a LIS3MDL magnetometer.
 */

 #include <LIS3MDL_Teensy4_I2C.h>

 // Create magnetometer instance
 LIS3MDL_Teensy4_I2C mag;
 
 // Sensor addresses
 #define MAG_ADDR 0x1C  // Default address (can be 0x1E depending on your hardware)
 
 // Timing variables
 unsigned long lastReadTime = 0;
 const unsigned long READ_INTERVAL = 100; // Read every 100ms
 
 void setup() {
   // Initialize serial communication
   Serial.begin(115200);
   
   // Wait for serial port to open
   while (!Serial) delay(10);
   
   Serial.println("\nLIS3MDL Magnetometer Example");
   Serial.println("=============================");
   
   // Initialize the magnetometer
   if (!mag.begin(MAG_ADDR)) {
     Serial.println("ERROR: Could not initialize magnetometer!");
     while (1) delay(10);
   }
   
   Serial.println("Magnetometer initialized successfully!");
   
   // Configure the magnetometer (optional - defaults are set in begin())
   mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
   mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
   mag.setRange(LIS3MDL_RANGE_4_GAUSS);
   mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
   
   // Print header for data output
   Serial.println("X_raw,Y_raw,Z_raw,X_gauss,Y_gauss,Z_gauss");
 }
 
 void loop() {
   unsigned long currentTime = millis();
   
   // Read data at regular intervals
   if (currentTime - lastReadTime >= READ_INTERVAL) {
     lastReadTime = currentTime;
     
     // Read data from the magnetometer
     mag.read();
     
     // Print raw data
     Serial.print(mag.x);
     Serial.print(",");
     Serial.print(mag.y);
     Serial.print(",");
     Serial.print(mag.z);
     Serial.print(",");
     
     // Print data in gauss
     Serial.print(mag.x_gauss, 6);
     Serial.print(",");
     Serial.print(mag.y_gauss, 6);
     Serial.print(",");
     Serial.println(mag.z_gauss, 6);
   }
 }