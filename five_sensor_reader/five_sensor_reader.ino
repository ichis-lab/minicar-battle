/*
 * five_sensor_reader.ino
 *
 * 5つのVL53L1Xセンサー測定テスト
 * 制御ロジックなし、センサー読み取りと詳細データ表示のみ
 *
 * 接続:
 * - TCA9548A I2Cマルチプレクサ (アドレス: 0x70)
 * - VL53L1Xセンサー → マルチプレクサのチャンネル0-4に接続
 *
 * 必要なライブラリ:
 * - VL53L1X (Pololu) - https://github.com/pololu/vl53l1x-arduino
 */

#include "Config.h"
#include "Logger.h"
#include "SensorReader.h"

// ============================================================================
// グローバルオブジェクト
// ============================================================================
SensorReader sensorReader;

// ============================================================================
// Setup関数
// ============================================================================
void setup() {
  // ロガー初期化（115200bpsで詳細データ表示に対応）
  Logger::begin(115200);

  Logger::println("==========================================");
  Logger::println("  VL53L1X 5-Sensor Test");
  Logger::println("==========================================");
  Logger::print("Measurement Interval: ");
  Logger::print(MEASUREMENT_INTERVAL);
  Logger::println("ms");
  Logger::print("L1X Timing Budget: ");
  Logger::print(L1X_TIMING_BUDGET_US / 1000);
  Logger::println("ms");
  Logger::println();

  // センサー初期化
  if (!sensorReader.begin()) {
    Logger::println("ERROR: Sensor initialization failed!");
    while (1) { delay(100); }  // 無限ループで停止
  }

  Logger::println();
  Logger::println("System ready! Starting measurements...");
  Logger::println();

  // ヘッダー表示
  Logger::println("S0(-70) | S1(-20) | S2(0) | S3(+20) | S4(+70)");
  Logger::println("--------|---------|-------|---------|--------");
}

// ============================================================================
// Loop関数
// ============================================================================
void loop() {
  static unsigned long lastMeasurement = 0;
  unsigned long currentTime = millis();

  // 指定した間隔で測定
  if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
    lastMeasurement += MEASUREMENT_INTERVAL;

    // =========================================================================
    // センサーデータ取得
    // =========================================================================
    sensorReader.readAll();
    const SensorData* sensorData = sensorReader.getAllData();

    // =========================================================================
    // 詳細データ表示
    // =========================================================================
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      // 距離とステータス
      if (sensorData[i].valid) {
        Logger::print(sensorData[i].distance);
        Logger::print("mm");
      } else {
        Logger::print("---");
      }
      Logger::print("(");
      Logger::print(sensorData[i].status);
      Logger::print(")");

      if (i < NUM_SENSORS - 1) {
        Logger::print(" | ");
      }
    }
    Logger::println();

    // 詳細データ行（信号強度・環境光）
    Logger::print("  sig:");
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      Logger::print(sensorData[i].peak_signal_mcps, 1);
      if (i < NUM_SENSORS - 1) Logger::print("/");
    }
    Logger::print(" amb:");
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      Logger::print(sensorData[i].ambient_mcps, 1);
      if (i < NUM_SENSORS - 1) Logger::print("/");
    }
    Logger::println();
  }
}
