/*
 * SensorReader.cpp
 *
 * センサー読み取りクラス（実装）
 */

#include "SensorReader.h"
#include "Logger.h"

SensorReader::SensorReader() {
  // 初期化
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    sensorData[i].distance = 0;
    sensorData[i].valid = false;
    sensorData[i].status = 255;
  }
}

void SensorReader::selectChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool SensorReader::begin() {
  Wire.begin();

  Logger::println("=== Sensor Initialization ===");

  // 各センサーを初期化
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    selectChannel(SENSOR_CHANNELS[i]);
    delay(10);  // チャンネル切替後の安定待ち

    Logger::print("Sensor ");
    Logger::print(i);
    Logger::print(" (Ch");
    Logger::print(SENSOR_CHANNELS[i]);
    Logger::print(", ");
    Logger::print(SENSOR_ANGLES[i]);
    Logger::print("deg)...");

    if (!sensors[i].begin()) {
      Logger::println(" FAILED!");
      return false;
    }

    Logger::println(" OK");
  }

  Logger::println("=== All sensors initialized ===");
  return true;
}

void SensorReader::readAll() {
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    selectChannel(SENSOR_CHANNELS[i]);
    sensors[i].rangingTest(&measurements[i], false);

    sensorData[i].status = measurements[i].RangeStatus;

    if (measurements[i].RangeStatus != 4) {
      sensorData[i].distance = measurements[i].RangeMilliMeter;
      sensorData[i].valid = true;
    } else {
      sensorData[i].distance = 0;
      sensorData[i].valid = false;
    }
  }
}

SensorData SensorReader::getSensorData(uint8_t index) const {
  if (index < NUM_SENSORS) {
    return sensorData[index];
  }
  SensorData empty = {0, false, 255};
  return empty;
}

const SensorData* SensorReader::getAllData() const {
  return sensorData;
}
