#include <Arduino.h>
#include <SPI.h>
#include "ad717x.hpp"

// Pin definitions - adjust if needed
const uint8_t ADC_CS_PIN = 10;   // ADC chip select pin
const uint8_t ADC_DRDY_PIN = 24;  // ADC data ready pin (interrupt)

// AD7175-8 driver
AD717X adcDriver;

// Interrupt flag
volatile bool dataReady = false;

// ISR for data ready pin
void adcInterrupt() {
  dataReady = true;
}

void printRegister(const char* name, uint8_t addr) {
  int result = adcDriver.readRegister(addr);
  ad717x_st_reg* reg = adcDriver.getReg(addr);
  
  if (result >= 0 && reg) {
    Serial.print(name);
    Serial.print(" (0x");
    Serial.print(addr, HEX);
    Serial.print("): 0x");
    Serial.println(reg->value, HEX);
  } else {
    Serial.print("Failed to read ");
    Serial.print(name);
    Serial.print(", error: ");
    Serial.println(result);
  }
}

void setup() {
  // Initialize serial and wait for connection
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("\n\n==========================================");
  Serial.println("AD7175-8 Standalone Test");
  Serial.println("==========================================");
  
  // Initialize SPI
  Serial.println("1. Initializing SPI");
  SPI.begin();
  
  // Set up pins
  Serial.println("2. Configuring pins");
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);  // Deselect ADC
  pinMode(ADC_DRDY_PIN, INPUT_PULLUP);
  
  // Perform manual reset sequence
  Serial.println("3. Performing manual reset sequence");
  digitalWrite(ADC_CS_PIN, LOW);
  
  // Send 8 bytes of 0xFF for reset
  for (int i = 0; i < 8; i++) {
    SPI.transfer(0xFF);
  }
  
  digitalWrite(ADC_CS_PIN, HIGH);
  delay(10); // Wait for ADC to reset
  
  // Initialize ADC
  Serial.println("4. Initializing AD7175-8");
  
  // Create initialization parameters
  ad717x_init_param_t initParam;
  initParam.active_device = ID_AD7175_8;
  initParam.mode = CONTINUOUS;
  initParam.stat_on_read_en = true;
  initParam.ref_en = true;  // Enable internal reference
  
  // Configure channel map (initially all disabled)
  Serial.println("5. Setting up channel map");
  initParam.chan_map.resize(16);
  for (int i = 0; i < 16; i++) {
    initParam.chan_map[i].channel_enable = false;
    initParam.chan_map[i].setup_sel = 0;
    initParam.chan_map[i].inputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
    initParam.chan_map[i].inputs.ainp.neg_input = AIN1;
  }
  
  // Enable channel 0 only for testing
  initParam.chan_map[0].channel_enable = true;
  
  // Create default setup
  Serial.println("6. Creating setup configuration");
  ad717x_setup_t setup;
  setup.setup.bi_polar = true;
  setup.setup.input_buff = true;
  setup.setup.ref_buff = true;
  setup.setup.ref_source = INTERNAL_REF;
  setup.filter_config.odr = SPS_1000;  // Lower data rate for testing
  setup.gain = 1.0;
  
  initParam.setups.push_back(setup);
  
  // Initialize the ADC driver
  Serial.println("7. Calling AD717X init function");
  int result = adcDriver.init(initParam, &SPI, ADC_CS_PIN);
  
  if (result < 0) {
    Serial.print("ADC initialization failed with error code: ");
    Serial.println(result);
    Serial.println("Halting execution");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("ADC initialization successful!");
  
  // Read and print registers to verify communication
  Serial.println("\n8. Reading ADC registers to verify communication:");
  
  printRegister("ID Register", AD717X_ID_REG);
  printRegister("Status Register", AD717X_STATUS_REG);
  printRegister("ADC Mode Register", AD717X_ADCMODE_REG);
  printRegister("Interface Mode Register", AD717X_IFMODE_REG);
  printRegister("GPIO Configuration Register", AD717X_GPIOCON_REG);
  printRegister("Channel Map 0 Register", AD717X_CHMAP0_REG);
  printRegister("Setup Config 0 Register", AD717X_SETUPCON0_REG);
  printRegister("Filter Config 0 Register", AD717X_FILTCON0_REG);
  
  // Set up interrupt for data ready
  Serial.println("\n9. Setting up DRDY interrupt");
  attachInterrupt(digitalPinToInterrupt(ADC_DRDY_PIN), adcInterrupt, CHANGE);
  
  // Start continuous conversion
  Serial.println("10. Starting continuous conversion");
  result = adcDriver.setADCMode(CONTINUOUS);
  
  if (result < 0) {
    Serial.print("Failed to start continuous conversion, error: ");
    Serial.println(result);
  } else {
    Serial.println("Continuous conversion started");
  }
  
  Serial.println("\nSetup complete - waiting for samples...");
  Serial.println("==========================================\n");
  dataReady = true;
}

void loop() {
  // Check if new data is ready
  if (dataReady) {
    dataReady = false;
    
    // Read data from ADC
    ad717x_data_t sample;
    int result = adcDriver.contConvReadData(&sample);
    
    if (result >= 0) {
      // Print sample information
      Serial.print("Channel: ");
      Serial.print(sample.status.active_channel);
      
      Serial.print(", Value: ");
      Serial.print(sample.value);
      
      // Calculate voltage (assuming 2.5V reference and 24-bit ADC)
      float voltage = (float)sample.value * 2.5 / 16777216.0; // 2^24 = 16,777,216
      Serial.print(", Voltage: ");
      Serial.print(voltage, 6);
      Serial.println(" V");
      
      // Additional status information
      if (sample.status.data_ready) Serial.println("  Data Ready: Yes");
      if (sample.status.adc_error) Serial.println("  ADC Error: Yes");
      if (sample.status.crc_error) Serial.println("  CRC Error: Yes");
      if (sample.status.reg_error) Serial.println("  Register Error: Yes");
    } else {
      Serial.print("Error reading ADC data: ");
      Serial.println(result);
    }
  }
  
  // Small delay to prevent flooding serial
  delay(10);
}