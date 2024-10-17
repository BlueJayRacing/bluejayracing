#define DEBUG_MODE 1  // Set to 0 for no debug, 1 for plotting only, 2 for all debug info
#define ENCODER_OPTIMIZE_INTERRUPTS  // Optional: for more optimized code

#include <limits>
#include <ADS1115_lite.h>
#include <TimeLib.h>
#include <SD.h>
#include <Encoder.h>

#if DEBUG_MODE > 0
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PLOT(x) x
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PLOT(x)
#endif

#if DEBUG_MODE > 1
  #define DEBUG_VERBOSE(x) x
#else
  #define DEBUG_VERBOSE(x)
#endif

const int chip_select = BUILTIN_SDCARD;
File file;
String curr_file_path;
int major_index;
int minor_index;
int orig_minute;
#define FILE_TIME 180000000
uint32_t file_diff;
uint32_t file_now;
ADS1115_lite adcs[4];
int16_t Raw;
bool hall1_flag = false;
bool hall2_flag = false;
int hall1_count = 0;
int hall2_count = 0;

// Encoder setup
const int encoderPinA = 36;
const int encoderPinB = 38;
const int encoderPinZ = 34;
Encoder myEncoder(encoderPinA, encoderPinB);
volatile bool zPulseDetected = false;
long encoderPosition = 0;

// Variables for periodic debug printing
#define DEBUG_PRINT_INTERVAL 200000  // 1/5 of a second in microseconds
uint32_t last_debug_print = 0;
int32_t channel_values[7] = {0};  // Array to store the latest values for all channels, including encoder

enum Channel {
  HALL1 = 0,
  HALL2 = 1,
  LIMPOT1 = 2,
  LIMPOT2 = 3,
  LIMPOT3 = 4,
  LIMPOT4 = 5,
  ENCODER = 6
};

void setup(void) {
  orig_minute = minute();
  
  #if DEBUG_MODE > 0
    Serial.begin(250000);
    while (!Serial) {
      ; // Wait for serial port to connect
    }
    DEBUG_VERBOSE(DEBUG_PRINTLN(F("Debug mode enabled")));
    DEBUG_PRINTLN(F("HALL1,HALL2,LIMPOT1,LIMPOT2,LIMPOT3,LIMPOT4,ENCODER,Z_PULSE"));
  #endif

  DEBUG_VERBOSE(DEBUG_PRINTLN(F("Initializing SD card...")));
  while (!SD.begin(chip_select)) {
    DEBUG_VERBOSE(DEBUG_PRINTLN("Could not open SD card"));
    delay(1000);
  }
  DEBUG_VERBOSE(DEBUG_PRINTLN(F("SD card initialized successfully")));

  major_index = read_meta_file_index() + 1;
  write_meta_file_index(major_index);
  minor_index = 0;
  curr_file_path = generate_file_path(major_index, minor_index);
  file = SD.open(curr_file_path.c_str(), FILE_WRITE);
  
  DEBUG_VERBOSE(DEBUG_PRINT(F("Opened initial file: ")));
  DEBUG_VERBOSE(DEBUG_PRINTLN(curr_file_path));

  for (int i = 0; i < 4; i++) {
    adcs[i] = *(new ADS1115_lite(0x48 + i));
    adcs[i].setGain(ADS1115_REG_CONFIG_PGA_6_144V);
    adcs[i].setSampleRate(ADS1115_REG_CONFIG_DR_128SPS);
    adcs[i].setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
    adcs[i].triggerConversion();
    DEBUG_VERBOSE(DEBUG_PRINT(F("Initialized ADS1115 #")));
    DEBUG_VERBOSE(DEBUG_PRINTLN(i));
  }

  attachInterrupt(digitalPinToInterrupt(13), hall_interrupt_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(41), hall_interrupt_2, FALLING);
  DEBUG_VERBOSE(DEBUG_PRINTLN(F("Attached hall effect sensor interrupts")));

  // Encoder setup
  pinMode(encoderPinZ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinZ), zPulseISR, RISING);
  DEBUG_VERBOSE(DEBUG_PRINTLN(F("Encoder initialized")));

  file_now = micros();
  file_diff = micros();

  DEBUG_VERBOSE(DEBUG_PRINTLN(F("Setup complete")));
}

void loop(void) {
  uint32_t current_time = micros();

  for (int i = 0; i < 4; i++) {
    int val = adcs[i].getConversion();
    adcs[i].triggerConversion();
    record(millis(), current_time, (Channel) (i + 2), val);
    channel_values[i + 2] = val;  // Store the latest value
  }

  if (hall1_flag) {
    hall1_flag = false;
    record(millis(), current_time, HALL1, hall1_count);
    channel_values[HALL1] = hall1_count;  // Store the latest value
  }

  if (hall2_flag) {
    hall2_flag = false;
    record(millis(), current_time, HALL2, hall2_count);
    channel_values[HALL2] = hall2_count;  // Store the latest value
  }

  // Read encoder
  long newPosition = myEncoder.read();
  if (newPosition != encoderPosition) {
    encoderPosition = newPosition;
    record(millis(), current_time, ENCODER, encoderPosition);
    channel_values[ENCODER] = encoderPosition;
    DEBUG_VERBOSE(DEBUG_PRINT(F("Encoder Position: ")));
    DEBUG_VERBOSE(DEBUG_PRINTLN(encoderPosition));
  }

  // Check for Z pulse
  if (zPulseDetected) {
    DEBUG_VERBOSE(DEBUG_PRINTLN(F("Z Pulse Detected!")));
    zPulseDetected = false;
  }

  // Check if it's time to print debug information
  if (current_time - last_debug_print >= DEBUG_PRINT_INTERVAL) {
    DEBUG_PLOT(print_debug_row(current_time));
    last_debug_print = current_time;
  }

  file_diff = current_time - file_now;
  if (file_diff > FILE_TIME) {
    file.close();
    DEBUG_VERBOSE(DEBUG_PRINT(F("Closing file: ")));
    DEBUG_VERBOSE(DEBUG_PRINTLN(curr_file_path));
    
    while (SD.exists(curr_file_path.c_str())) {
      curr_file_path = generate_file_path(major_index, ++minor_index);
    }
    
    file = SD.open(curr_file_path.c_str(), FILE_WRITE);
    DEBUG_VERBOSE(DEBUG_PRINT(F("Opened new file: ")));
    DEBUG_VERBOSE(DEBUG_PRINTLN(curr_file_path));
    
    file_now = current_time;
  }
}

void record(uint32_t time_milli, uint32_t time_micro, Channel channel, int32_t value) {
  file.print(time_milli);
  file.print(F(","));
  file.print(time_micro);
  file.print(F(","));
  file.print(channel);
  file.print(F(","));
  file.print(value);
  file.println(F(","));
}

void print_debug_row(uint32_t current_time) {
  #if DEBUG_MODE > 0
    DEBUG_VERBOSE(DEBUG_PRINT(current_time));
    DEBUG_VERBOSE(DEBUG_PRINT(F(",")));
    for (int i = 0; i < 7; i++) {  // Increased to 7 to include ENCODER
      DEBUG_PRINT(channel_values[i]);
      DEBUG_PRINT(F(","));
    }
    DEBUG_PRINT(zPulseDetected ? F("1") : F("0"));  // Print Z pulse status
    DEBUG_PRINTLN();
  #endif
}

void hall_interrupt_1() {
  hall1_count++;
  hall1_flag = true;
}

void hall_interrupt_2() {
  hall2_count++;
  hall2_flag = true;
}

// Interrupt Service Routine for Z channel
void zPulseISR() {
  zPulseDetected = true;
}

String generate_file_path(int major_index, int minor_index) {
  String major_string = String(major_index);
  while (major_string.length() < 3) {
    major_string = "0" + major_string;
  }
  String minor_string = String(minor_index);
  while (minor_string.length() < 3) {
    minor_string = "0" + minor_string;
  }
  return major_string + "_" + minor_string + "_butler.txt";
}

int read_meta_file_index(void) {
  if (!SD.exists("metadata.txt")) {
    write_meta_file_index(0);
    DEBUG_VERBOSE(DEBUG_PRINTLN(F("Created new metadata file")));
    return -1;
  }
  File meta_file = SD.open("metadata.txt", FILE_READ);
  String line = meta_file.readStringUntil('\n');
  int index = line.toInt();
  meta_file.close();
  DEBUG_VERBOSE(DEBUG_PRINT(F("Read meta file index: ")));
  DEBUG_VERBOSE(DEBUG_PRINTLN(index));
  return index;
}

void write_meta_file_index(int index) {
  File meta_file = SD.open("metadata.txt", FILE_WRITE_BEGIN);
  meta_file.println(index);
  meta_file.close();
  DEBUG_VERBOSE(DEBUG_PRINT(F("Wrote meta file index: ")));
  DEBUG_VERBOSE(DEBUG_PRINTLN(index));
}
