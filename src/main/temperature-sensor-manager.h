#ifndef _TEMPERATURE_SENSOR_MANAGER_H_
#define _TEMPERATURE_SENSOR_MANAGER_H_

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "utils.h"

class TemperatureSensor {
public:
  TemperatureSensor(OneWire &oneWire)
    : sensors(&oneWire) {
    sensors.begin();
  }

  float readTemperature() {
    sensors.requestTemperatures();
    return sensors.getTempCByIndex(0);
  }

private:
  DallasTemperature sensors;
};

class TemperatureDevice {
public:
  TemperatureDevice(int oneWireBus)
    : oneWire(oneWireBus), temperatureSensor(oneWire) {}

  static void RunTask(void *pvParameters) {
    TemperatureDevice *temperatureDevice = static_cast<TemperatureDevice *>(pvParameters);
    temperatureDevice->run();
  }

  void run() {
    for (;;) {
      Serial.println("TaskTemperatureDevice start");
      uint32_t deviceId = SimpleUtils::getDeviceId();
      float temperature = temperatureSensor.readTemperature();
      char datetime[64];
      SimpleUtils::getCurrentTime(datetime);
      Serial.print(datetime);
      Serial.print(" - ");
      Serial.print(temperature);
      Serial.println("ºC");
      // metricsPost(String(deviceId), datetime, "T", temperature);
      MessageManager &messageManager = MessageManager::getInstance();
      messageManager.publishMessage(String(deviceId), datetime, "T", temperature);
      Serial.println("TaskTemperatureDevice end");
      delay(1000 * 60);
      // delay(1000);
    }
  }

private:
  OneWire oneWire;
  TemperatureSensor temperatureSensor;
};


#endif
