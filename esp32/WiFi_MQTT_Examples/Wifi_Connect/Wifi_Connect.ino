#include<WiFi.h>

const char *ssid = "piWifi";  
const char *password = "bluejayracing";
        
void initWiFi() {
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      Serial.print("Connecting to WiFi ..");
      while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
      }
      Serial.println(WiFi.localIP());
    }
    
    void setup() {
      Serial.begin(115200);
      initWiFi();
      Serial.print("RRSI: ");
      Serial.println(WiFi.RSSI());
    }
    
    void loop() {
      // put your main code here, to run repeatedly:
    }