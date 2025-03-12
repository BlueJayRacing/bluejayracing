#include <Arduino.h>
#include <SPI.h>
#include "ad717x.hpp"

// Pin definitions
#define ADC_DRDY_PIN       24    // Dedicated DRDY input (buffered from ADC's DRDY/MISO pin)
#define ADC_CS_PIN         10    // Chip Select for ADC
#define PRINT_INTERVAL_MS  300   // 300 ms averaging period

// Global ADC driver instance and initialization parameters
AD717X adcDriver;
ad717x_init_param_t initParams;

// Interrupt flag: set when DRDY goes low
volatile bool dataReadyFlag = false;

// Arrays to accumulate conversion sums and counts for 16 channels
volatile uint64_t channelSum[16];
volatile uint32_t channelCount[16];

// For monitoring overall conversion count (optional)
volatile uint64_t totalSampleCount = 0;

uint32_t lastPrintTime = 0;
bool headerPrinted = false;

// ISR: only sets the flag (keep it minimal)
void drdyISR() {
  dataReadyFlag = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to initialize

  // Print CSV header (only once)
  Serial.println("Rate,Ch0,Ch1,Ch2,Ch3,Ch4,Ch5,Ch6,Ch7,Ch8,Ch9,Ch10,Ch11,Ch12,Ch13,Ch14,Ch15");
  headerPrinted = true;

  // Initialize accumulators to zero
  for (int i = 0; i < 16; i++) {
    channelSum[i] = 0;
    channelCount[i] = 0;
  }

  // Initialize SPI (use a moderate clock speed, e.g. 5MHz)
  SPI.begin();
  
  // Configure chip-select pin for ADC
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);  // Deselect ADC

  // Configure the DRDY pin; ensure it’s properly buffered
  pinMode(ADC_DRDY_PIN, INPUT_PULLUP);

  // Set up ADC initialization parameters:
  // - Use continuous conversion mode.
  // - Enable status on read (so each conversion result includes channel info).
  // - Use external reference (adjust if needed).
  initParams.active_device = ID_AD7175_8;
  initParams.mode = CONTINUOUS;
  initParams.stat_on_read_en = true;
  initParams.ref_en = true;

  // Create one default setup configuration.
  ad717x_setup_t setupConfig;
  setupConfig.setup.bi_polar = false;
  setupConfig.setup.input_buff = true;
  setupConfig.setup.ref_buff = true;
  setupConfig.setup.ref_source = EXTERNAL_REF;
  // Choose an output data rate; for example, SPS_2500 (adjust as needed)
  setupConfig.filter_config.odr = SPS_5;
  setupConfig.gain = 1.0;
  initParams.setups.push_back(setupConfig);

  // Configure channel mapping for all 16 channels.
  initParams.chan_map.resize(16);
  for (int i = 0; i < 16; i++) {
    initParams.chan_map[i].channel_enable = true;  // Enable every channel
    initParams.chan_map[i].setup_sel = 0;            // All channels use setup 0
    // Configure inputs: assume single-ended; positive input is channel number, negative tied to REF_M.
    initParams.chan_map[i].inputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
    initParams.chan_map[i].inputs.ainp.neg_input = REF_M;
  }

  // Initialize the ADC (using a 5MHz SPI clock).
  int initResult = adcDriver.init(initParams, &SPI, ADC_CS_PIN, SPISettings(50000000, MSBFIRST, SPI_MODE3));
//   Serial.print("ADC init result: ");
//   Serial.println(initResult);
  if (initResult < 0) {
    Serial.println("ADC initialization failed. Check wiring and SPI settings.");
    while (1);
  }

  // Optionally, read back the ID register to verify communication.
  ad717x_st_reg_t* idReg = adcDriver.getReg(AD717X_ID_REG);
  if (idReg) {
    // Serial.print("ADC ID register value: 0x");
    // Serial.println(idReg->value, HEX);
  } else {
    Serial.println("Could not read ADC ID register.");
  }

  ad717x_st_reg_t* filtReg = adcDriver.getReg(AD717X_FILTCON0_REG);
  if (filtReg) {
    // Print in hex:
    Serial.print("FILTCON0 register (raw) = 0x");
    Serial.println(filtReg->value, HEX);

    // Print as a 32-bit binary:
    uint32_t regVal = (uint32_t)filtReg->value & 0xFFFFFFFFUL;
    Serial.print("FILTCON0 register (32-bit binary) = ");
    for (int i = 31; i >= 0; i--) {
      Serial.print((regVal >> i) & 1);
      // Optional spacing every 4 bits:
      if (i % 4 == 0) {
        Serial.print(" ");
      }
    }
    Serial.println();
  } else {
    Serial.println("Could not read FILTCON0 register.");
  }


  // Attach an interrupt to the dedicated DRDY pin (FALLING edge triggers when conversion is complete).
  attachInterrupt(digitalPinToInterrupt(ADC_DRDY_PIN), drdyISR, FALLING);

  lastPrintTime = millis();

  delay(1);
  dataReadyFlag = true;

}

void loop() {
  // Process ADC conversion results whenever the interrupt flag is set.
  if (dataReadyFlag) {
    noInterrupts();
    dataReadyFlag = false;
    interrupts();

    ad717x_data_t sample;
    int result = adcDriver.contConvReadData(&sample);
    if (result >= 0) {
      uint8_t ch = sample.status.active_channel; // Determine which channel this conversion is for
      if (ch < 16) {  // safeguard in case channel index is out-of-range
        channelSum[ch] += sample.value;
        channelCount[ch]++;
      }
      totalSampleCount++;
    }
    // If result < 0, you could record an error (not shown here)
  }

  uint32_t currentMillis = millis();
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL_MS) {
    // Create temporary arrays to hold sums and counts while resetting global accumulators.
    uint64_t tempSum[16];
    uint32_t tempCount[16];
    uint32_t totalCount = 0;
    for (int i = 0; i < 16; i++) {
      noInterrupts();
      tempSum[i] = channelSum[i];
      tempCount[i] = channelCount[i];
      channelSum[i] = 0;
      channelCount[i] = 0;
      interrupts();
      totalCount += tempCount[i];
    }
    // Compute a per-channel rate (samples per second) as an average over channels.
    double rate = 0.0;
    if (16 > 0) {
      rate = ((double)totalCount) * (1000.0 / PRINT_INTERVAL_MS);
    }

    // Compute average conversion value for each channel.
    double averages[16];
    for (int i = 0; i < 16; i++) {
      if (tempCount[i] > 0)
        averages[i] = (double)tempSum[i] / tempCount[i];
      else
        averages[i] = 0;
    }

    // Print CSV line: first column is the average rate, then one column per channel.
    Serial.print(rate, 2);
    for (int i = 0; i < 16; i++) {
      Serial.print(",");
      Serial.print(averages[i], 2);
    }
    Serial.println();

    lastPrintTime = currentMillis;
  }
}
