#include <Arduino.h>
#include <SPI.h>
#include "ad717x.hpp"

// Pin definitions
#define ADC_CS_PIN         10    // Chip Select for ADC
#define PRINT_INTERVAL_MS  500   // Interval for printing results

// AVDD-AVSS reference voltage (measured at 3.4V)
#define VREF               5   // Replace with your actual measured value

// Global ADC driver instance
AD717X adcDriver;

// Arrays to accumulate conversion sums and counts for 16 channels
uint64_t channelSum[16];
uint32_t channelCount[16];

uint32_t lastPrintTime = 0;
uint32_t totalSamples = 0;

// Channel names for better readability
const char* channelNames[] = {
  "AIN0 (Pin 2)",
  "AIN1/REF2+ (5v_reful)",
  "AIN2",
  "AIN3",
  "AIN4 (2v5_ref)",
  "AIN5 (2v5_reful)",
  "REFOUT",
  "AIN7"
};

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait for Serial or timeout

  Serial.println("\nAD7175-8 Configuration with 3.4V Reference");
  Serial.println("=========================================");

  // Initialize accumulators to zero
  for (int i = 0; i < 16; i++) {
    channelSum[i] = 0;
    channelCount[i] = 0;
  }

  // Initialize SPI
  SPI.begin();
  
  // Configure chip-select pin for ADC
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);  // Deselect ADC

  // Reset the ADC
  Serial.println("Resetting ADC...");
  adcDriver.reset();
  delay(50);  // Allow time for reset to complete

  // Basic ADC configuration
  ad717x_init_param_t initParams;
  initParams.active_device = ID_AD7175_8;
  initParams.mode = CONTINUOUS;
  initParams.stat_on_read_en = true;  // Important to get channel info
  initParams.ref_en = false;           // Enable reference

  // Create setup that uses AVDD-AVSS (3.4V) as reference
  ad717x_setup_t setup0;
  setup0.setup.bi_polar = false;            // Unipolar mode
  setup0.setup.input_buff = true;           // Enable input buffer
  setup0.setup.ref_buff = false;            // Disable reference buffer
  setup0.setup.ref_source = AVDD_AVSS;      // Use AVDD-AVSS (3.4V) as reference
  setup0.filter_config.odr = SPS_50;        // 50 samples per second
  setup0.gain = 1.0;                        // Unity gain
  initParams.setups.push_back(setup0);

  // Configure channel mappings
  initParams.chan_map.resize(16);
  
  // Configure first 6 channels with AVSS as negative input
  for (int i = 0; i < 6; i++) {
    initParams.chan_map[i].channel_enable = true;
    initParams.chan_map[i].setup_sel = 0;  // Use setup 0 (AVDD-AVSS)
    initParams.chan_map[i].inputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
    initParams.chan_map[i].inputs.ainp.neg_input = AVDD_AVSS_M;  // Use AVSS as negative input
  }

  // Disable other channels
  for (int i = 6; i < 16; i++) {
    initParams.chan_map[i].channel_enable = false;
  }

  // Initialize the ADC
  Serial.println("Initializing ADC...");
  int initResult = adcDriver.init(initParams, &SPI, ADC_CS_PIN);
  if (initResult < 0) {
    Serial.println("ADC initialization failed. Check wiring and SPI settings.");
    while (1); // Stop if initialization failed
  } else {
    Serial.println("ADC initialized successfully.");
  }
  
  // Manually verify key register settings
  ad717x_st_reg* adcModeReg = adcDriver.getReg(AD717X_ADCMODE_REG);
  if (adcModeReg) {
    // Force REF_EN bit
    adcModeReg->value |= AD717X_ADCMODE_REG_REF_EN;
    adcDriver.writeRegister(AD717X_ADCMODE_REG);
    
    // Read back to verify
    adcDriver.readRegister(AD717X_ADCMODE_REG);
    Serial.print("ADC Mode Register: 0x");
    Serial.println(adcModeReg->value, HEX);
    Serial.print("Reference Enable: ");
    Serial.println((adcModeReg->value & AD717X_ADCMODE_REG_REF_EN) ? "YES" : "NO");
  }

  // Verify setup 0 register
  ad717x_st_reg* setupReg = adcDriver.getReg(AD717X_SETUPCON0_REG);
  if (setupReg) {
    // Read to verify
    adcDriver.readRegister(AD717X_SETUPCON0_REG);
    Serial.print("Setup 0 Register: 0x");
    Serial.println(setupReg->value, HEX);
    
    // Check reference source
    uint8_t refSource = (setupReg->value & AD717X_SETUP_CONF_REG_REF_SEL(3)) >> 4;
    Serial.print("Reference Source: ");
    switch (refSource) {
      case 0: Serial.println("External REFIN1(+)/REFIN1(-)"); break;
      case 1: Serial.println("External REFIN2(+)/REFIN2(-)"); break;
      case 2: Serial.println("Internal Reference"); break;
      case 3: Serial.println("AVDD-AVSS"); break;
      default: Serial.println("Unknown"); break;
    }
  }

  Serial.print("Using reference voltage: ");
  Serial.print(VREF);
  Serial.println("V");

  // Print CSV header
  Serial.println("\nChannel,Name,Raw Value,Voltage");
  
  lastPrintTime = millis();
}

void loop() {
  // Poll for ADC data using waitForReady
  if (adcDriver.waitForReady(0) == 0) {  // This will return immediately if data is ready
    // Read ADC data with status
    ad717x_data_t sample;
    int result = adcDriver.contConvReadData(&sample);
    
    if (result >= 0) {
      uint8_t ch = sample.status.active_channel;
      if (ch < 16) {
        // Accumulate the reading
        channelSum[ch] += sample.value;
        channelCount[ch]++;
        totalSamples++;
      }
    }
  }

  // Print results periodically
  uint32_t currentMillis = millis();
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL_MS) {
    // Print sample rate
    float sampleRate = (float)totalSamples * 1000.0 / PRINT_INTERVAL_MS;
    Serial.print("Sample rate: ");
    Serial.print(sampleRate, 1);
    Serial.println(" SPS");
    totalSamples = 0;
    
    // Print raw values and voltages for active channels
    for (int i = 0; i < 6; i++) {
      if (channelCount[i] > 0) {
        double rawValue = (double)channelSum[i] / channelCount[i];
        
        // Calculate voltage using the actual VREF (3.4V)
        double voltage = rawValue / 16777216.0 * VREF;
        
        // Calculate what the 5V equivalent would be
        double voltage5V = (voltage / VREF) * 5.0;
        
        Serial.print(i);
        Serial.print(",");
        Serial.print(channelNames[i]);
        Serial.print(",");
        Serial.print(rawValue, 0);  // Raw ADC value
        Serial.print(",");
        Serial.print(voltage, 4);   // Actual voltage with 3.4V reference
        Serial.print(",");
        Serial.println(voltage5V, 4); // Equivalent voltage with 5V reference (for comparison)
      }
    }
    
    // Reset accumulators after printing
    for (int i = 0; i < 16; i++) {
      channelSum[i] = 0;
      channelCount[i] = 0;
    }
    
    // Add a separator between readings
    Serial.println("---");
    
    lastPrintTime = currentMillis;
  }
}