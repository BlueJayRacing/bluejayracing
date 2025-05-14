/****************************************************************************************************************************
  Async_UdpAsyncCallExperiment.ino

  For Teensy 4.1 with QNEthernet

  This sketch runs an experiment to determine the overhead time of making an asynchronous UDP send call 
  (using udp.write from AsyncUDP_Teensy41) as a function of packet size and sending rate.
  
  For each combination of packet size and rate, it sends 100 packets, measures the time spent in calling 
  udp.write(), and then prints the average overhead in microseconds along with the actual message send rate.
  
  Built by [Your Name] based on AsyncUDP_Teensy41 and QNEthernet.
*****************************************************************************************************************************/

#include <Arduino.h>
#include <QNEthernet.h>            // Use QNEthernet for Teensy 4.1
#include <AsyncUDP_Teensy41.h>      // Async UDP library for Teensy 4.1 using QNEthernet

// Use DHCP for network configuration
#define USING_DHCP 1

#if !USING_DHCP
IPAddress myIP(192, 168, 20, 10);
IPAddress myNetmask(255, 255, 255, 0);
IPAddress myGW(192, 168, 20, 1);
#endif

// Target UDP server settings
IPAddress serverIP(192, 168, 20, 3);  // This is the ROS2 server IP
const uint16_t serverPort = 8888;      // UDP port

// Create an AsyncUDP instance.
AsyncUDP udp;

// Define arrays for packet sizes and sending rates.
const int numPacketSizes = 6;
const int packetSizes[numPacketSizes] = {64, 128, 256, 512, 1024, 1472};

const int numRates = 7;
const int rates[numRates] = {10, 50, 100, 500, 1000, 10000, 100000};  // messages per second

// Number of iterations (packets) per experiment.
const int iterations = 100;

// This function runs the experiment for each packet size and rate combination.
void runExperiment() {
  Serial.println("Starting async call overhead experiment...");
  
  // Loop over each packet size.
  for (int i = 0; i < numPacketSizes; i++) {
    int size = packetSizes[i];
    // Create a payload buffer filled with 'A's.
    char buf[size];
    memset(buf, 'A', size);
    
    // Loop over each sending rate.
    for (int j = 0; j < numRates; j++) {
      int rate = rates[j];
      unsigned long totalOverhead = 0;
      // Calculate inter-packet delay in microseconds.
      unsigned long interval_us = 1000000UL / rate;
      
      // Measure the start time for the batch.
      unsigned long experimentStartTime = micros();
      
      // Send a batch of packets.
      for (int k = 0; k < iterations; k++) {
        unsigned long startTime = micros();
        udp.write(buf, size);
        unsigned long endTime = micros();
        totalOverhead += (endTime - startTime);
        
        // Enforce the sending rate.
        unsigned long callTime = (endTime - startTime);
        if (interval_us > callTime) {
          delayMicroseconds(interval_us - callTime);
        }
      }
      
      // Measure the end time for the batch.
      unsigned long experimentEndTime = micros();
      unsigned long totalExperimentTime = experimentEndTime - experimentStartTime;
      // Compute the actual message send rate.
      float actualRate = iterations * 1000000.0 / totalExperimentTime;
      
      float avgOverhead = totalOverhead / (float)iterations;
      Serial.print("Packet Size: ");
      Serial.print(size);
      Serial.print(" bytes, Target Rate: ");
      Serial.print(rate);
      Serial.print(" msg/s, Avg async call overhead: ");
      Serial.print(avgOverhead);
      Serial.print(" us, Actual send rate: ");
      Serial.print(actualRate);
      Serial.println(" msg/s");
    }
  }
  Serial.println("Experiment complete.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial monitor

  Serial.println("Async UDP Async Call Experiment Sketch Starting...");
  
#if USING_DHCP
  Serial.println("Initializing QNEthernet using DHCP...");
  Ethernet.begin();
#else
  Serial.println("Initializing QNEthernet using static IP...");
  QNEthernet.begin(myIP, myNetmask, myGW);
#endif

  // Wait up to 5000 ms for a valid IP.
  if (!Ethernet.waitForLocalIP(5000)) {
    Serial.println("Failed to obtain IP address. Check your network connection.");
    while (true) delay(1);
  }
  
  Serial.print("Local IP: ");
  Serial.println(Ethernet.localIP());
  
  // Connect the UDP object to the target server.
  if (udp.connect(serverIP, serverPort)) {
    Serial.println("UDP connected to server.");
  } else {
    Serial.println("Failed to connect UDP to server.");
  }
  
  // Optional: set up a minimal onPacket callback (not used in this experiment).
  udp.onPacket([](AsyncUDPPacket packet) {
    // No action needed; experiment focuses on async call overhead.
  });
  
  // Run the experiment once.
  runExperiment();
}

void loop() {
  // No periodic action required; experiment is run once in setup().
}
