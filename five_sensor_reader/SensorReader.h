/*
 * SensorReader.h
 *
 * センサー読み取りクラス（宣言）
 * TCA9548A経由でVL53L0Xセンサーからデータ取得
 */

#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Config.h"

// センサーデータ構造体
struct SensorData {
  uint16_t distance;  // 測定距離（mm）
  bool valid;         // 測定値の有効性
  uint8_t status;     // VL53L0Xステータスコード
};

// センサーリーダークラス
class SensorReader {
private:
  Adafruit_VL53L0X _sensors[NUM_SENSORS];
  VL53L0X_RangingMeasurementData_t _measurements[NUM_SENSORS];
  SensorData _sensorData[NUM_SENSORS];

  // TCA9548Aのチャンネル選択
  void _selectChannel(uint8_t channel);

public:
  // コンストラクタ
  SensorReader();

  // 初期化
  bool begin();

  // 全センサーからデータ取得
  void readAll();

  // 指定センサーのデータ取得
  SensorData getSensorData(uint8_t index) const;

  // 全センサーデータの配列を取得
  const SensorData* getAllData() const;
};

#endif // SENSOR_READER_H
