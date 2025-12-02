#include "WString.h"
#ifndef _MESSAGE_MANAGER_H_
#define _MESSAGE_MANAGER_H_

#include <WiFi.h>
#include <MQTT.h>
#include "utils.h"

// 测试环境
// #define MESSAGE_TOPIC_PRIFIX "/GgkdHit3oYTChLQC/"
// 生产环境
#define MESSAGE_TOPIC_PRIFIX "/ykGtJMDoqtbuyLTt/"

class MessageManager {
public:
  static MessageManager &getInstance() {
    static MessageManager instance;
    return instance;
  }

  void setWifi(WiFiClient &wifi) {
    mqttClient.begin("broker.emqx.io", wifi);
    connectIfNecessary();
  }

  bool publishMessage(String deviceId, String metricTime, String metricType, float metricValue) {
    String payload = deviceId + "\n0\n" + metricTime + "\n" + metricType + "\n" + String(metricValue, 2);
    return mqttClient.publish(String(MESSAGE_TOPIC_PRIFIX) + "metrics", payload);
  }

  void loop() {
    mqttClient.loop();
    connectIfNecessary();
  }

private:
  MQTTClient mqttClient;

  MessageManager() {
    initMQTTClient();
    initMessageManagerLoopTask();
  }

  MessageManager(const MessageManager &) = delete;
  MessageManager(const MessageManager &&) = delete;
  MessageManager &operator=(const MessageManager &) = delete;

  void initMQTTClient() {
    Serial.println("MessageManager Object Created");
    mqttClient.onMessage(MessageManager::messageReceived);
  }

  void initMessageManagerLoopTask() {
    xTaskCreate(
      MessageManager::messageManagerLoopTask,
      "messageManagerLoopTask",
      2048,
      NULL,
      2,
      NULL);
  }

  static void messageReceived(String &topic, String &payload) {
    Serial.println("incoming: " + topic + " - " + payload);
    // Do not use the client in the callback to publish, subscribe, or unsubscribe.
    // Instead, change a global variable or push to a queue and handle it in the loop.
  }

  static void messageManagerLoopTask(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
      delay(1000);
      MessageManager &messageManager = MessageManager::getInstance();
      messageManager.loop();
    }
  }

  void connectIfNecessary() {
    if (mqttClient.connected()) {
      return;
    }

    Serial.print("Checking WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(1000);
    }

    Serial.println("\nWiFi connected");
    Serial.print("Message client connecting...");

    String deviceId = String(SimpleUtils::getDeviceId());
    char clientID[deviceId.length()];
    strcpy(clientID, deviceId.c_str());

    while (!mqttClient.connect(clientID)) {
      Serial.print(".");
      delay(1000);
    }

    Serial.println("\nMessage client connected!");

    mqttClient.subscribe(String(MESSAGE_TOPIC_PRIFIX) + "settings");
    mqttClient.subscribe(String(MESSAGE_TOPIC_PRIFIX) + String(deviceId) + "/settings");
  }
};

#endif
