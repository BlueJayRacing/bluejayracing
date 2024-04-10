Last Updated by Travis Chen on 3/27/2024

This is the wireless strain gauge Arduino IDE code for the XIAO ESP32C3 that communicates via MQTT to a central Raspberry Pi.

Board Manager:
- esp32 by Espressif Systems (ver. 2.0.11)

The following Arduino IDE libaries need to be installed via ZIP; to install, download the ZIP files of the libraries and add them to
Arduino IDE through Sketch > Manage Libraries > Add .ZIP Library.
- https://github.com/bertmelis/espMqttClient
- https://github.com/LucasEtchezuri/Arduino-ADS1120

The following Arduino IDE libraries need to be installed, and that can be done through the Library Manager:
- CleanRTOS by Marius Versteegen (ver. 0.08)
- Adafruit BusIO by Adafruit (ver. 1.15.0)
- AsyncTCP by dvarrel (ver. 1.1.4)
- ESPAsyncTCP by dvarrel (ver. 1.2.4)

Add the following link to the "Additional Board Manager URLS" as well
- https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json