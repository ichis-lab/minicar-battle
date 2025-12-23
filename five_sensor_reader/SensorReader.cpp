/*
 * SensorReader.cpp
 *
 * センサー読み取りクラス（実装）
 * VL53L1X対応版
 */

#include "SensorReader.h"
#include "Logger.h"

SensorReader::SensorReader() {
  // 初期化
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    _sensorData[i].distance = 0;
    _sensorData[i].valid = false;
    _sensorData[i].status = 255;
    _sensorData[i].peak_signal_mcps = 0.0;
    _sensorData[i].ambient_mcps = 0.0;
  }
}

void SensorReader::_selectChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool SensorReader::begin() {
  Wire.begin();
  Wire.setClock(400000);  // I2C高速モード（400kHz）

  Logger::println("=== VL53L1X Sensor Initialization ===");

  // 各センサーを初期化
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    _selectChannel(SENSOR_CHANNELS[i]);
    delay(10);  // チャンネル切替後の安定待ち

    Logger::print("Sensor ");
    Logger::print(i);
    Logger::print(" (Ch");
    Logger::print(SENSOR_CHANNELS[i]);
    Logger::print(", ");
    Logger::print(SENSOR_ANGLES[i]);
    Logger::print("deg)...");

    _sensors[i].setTimeout(500);
    if (!_sensors[i].init()) {
      Logger::println(" FAILED!");
      return false;
    }

    // VL53L1X設定: 長距離モード、測定時間50ms
    _sensors[i].setDistanceMode(VL53L1X::Long);
    _sensors[i].setMeasurementTimingBudget(L1X_TIMING_BUDGET_US);

    // 連続測定モード開始
    _sensors[i].startContinuous(L1X_INTER_MEASUREMENT_MS);

    Logger::println(" OK");
  }

  Logger::println("=== All VL53L1X sensors initialized ===");
  return true;
}

void SensorReader::readAll() {
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    _selectChannel(SENSOR_CHANNELS[i]);

    // VL53L1Xから読み取り（連続測定モード）
    _sensors[i].read();

    // 詳細データを取得
    _sensorData[i].distance = _sensors[i].ranging_data.range_mm;
    _sensorData[i].status = _sensors[i].ranging_data.range_status;
    _sensorData[i].peak_signal_mcps = _sensors[i].ranging_data.peak_signal_count_rate_MCPS;
    _sensorData[i].ambient_mcps = _sensors[i].ranging_data.ambient_count_rate_MCPS;

    // ステータスが "range valid" (0) なら有効
    // VL53L1X::rangeStatusToString() で確認可能
    _sensorData[i].valid = (_sensorData[i].status == 0);
  }
}

SensorData SensorReader::getSensorData(uint8_t index) const {
  if (index < NUM_SENSORS) {
    return _sensorData[index];
  }
  SensorData empty = {0, false, 255, 0.0, 0.0};
  return empty;
}

const SensorData* SensorReader::getAllData() const {
  return _sensorData;
}
