/*
 * SensorReader.cpp
 *
 * センサー読み取りクラス（実装）
 * 高速化対応版：timing budget設定と連続測定モード
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

    // 高速化: Timing Budget を設定 (20ms = 理論上50Hz per sensor)
    // High Speed モードに設定
    sensors[i].setMeasurementTimingBudgetMicroSeconds(SENSOR_TIMING_BUDGET);

    // 連続測定モード開始
    sensors[i].startRangeContinuous();

    Logger::println(" OK (High Speed)");
  }

  Logger::println("=== All sensors initialized (25Hz mode) ===");
  return true;
}

void SensorReader::readAll() {
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    selectChannel(SENSOR_CHANNELS[i]);

    // 連続モードでの読み取り
    if (sensors[i].isRangeComplete()) {
      uint16_t range = sensors[i].readRange();
      uint8_t status = sensors[i].readRangeStatus();

      sensorData[i].status = status;

      if (status == 0) {  // 0 = valid measurement
        sensorData[i].distance = range;
        sensorData[i].valid = true;
      } else {
        sensorData[i].distance = 0;
        sensorData[i].valid = false;
      }
    }
    // 測定完了していない場合は前回の値を保持
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
