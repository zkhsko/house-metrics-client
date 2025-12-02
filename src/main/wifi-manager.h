#ifndef _WIFI_MANAGER_H_
#define _WIFI_MANAGER_H_

#include <WiFi.h>
#include <SPI.h>
#include <MQTT.h>

// #define WIFI_SSID "TP-LINK_1755"
// #define WIFI_PASSWORD "1234567890"

// #define WIFI_SSID "rigongyizu"
// #define WIFI_PASSWORD "xiaozu100000"

// #define WIFI_SSID "TP-LINK_F65D"
// #define WIFI_PASSWORD "lidongpo"

#define WIFI_SSID "HUAWEI-F161"
#define WIFI_PASSWORD "lidongpo"

WiFiClient net;

class WifiManager {
public:

  static void startup() {
    // connect to WiFi
    Serial.printf("Connecting to %s ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println(" CONNECTED ");
    Serial.println(WiFi.localIP());

    MessageManager &messageManager = MessageManager::getInstance();
    messageManager.setWifi(net);

    // WiFi.disconnect(true);
    // WiFi.mode(WIFI_OFF);
  }

private:
  WifiManager() {}
};

#endif