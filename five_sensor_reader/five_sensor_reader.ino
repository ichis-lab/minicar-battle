/*
 * five_sensor_reader.ino
 *
 * 5つのVL53L1Xセンサーを使ったシンプル状態ベース制御
 *
 * 接続:
 * - TCA9548A I2Cマルチプレクサ (アドレス: 0x70)
 * - VL53L1Xセンサー → マルチプレクサのチャンネル0-4に接続
 * - サーボモーター → Pin 9
 * - ESC → Pin 10
 *
 * 必要なライブラリ:
 * - VL53L1X (Pololu) - https://github.com/pololu/vl53l1x-arduino
 * - Servo (Arduino標準ライブラリ)
 */

#include "Config.h"
#include "Logger.h"
#include "SensorReader.h"
#include "SteeringController.h"
#include "Actuator.h"

// ============================================================================
// グローバルオブジェクト
// ============================================================================
SensorReader sensorReader;
SteeringController steeringController;
Actuator actuator;

// ============================================================================
// Setup関数
// ============================================================================
void setup() {
  // ロガー初期化（115200bpsで詳細データ表示）
  Logger::begin(115200);

  Logger::println("==========================================");
  Logger::println("  VL53L1X Simple State-Based Control");
  Logger::println("==========================================");
  Logger::print("Debug Mode: ");
  Logger::println(DEBUG_MODE ? "ON (No PWM)" : "OFF (PWM Active)");
  Logger::print("Measurement Interval: ");
  Logger::print(MEASUREMENT_INTERVAL);
  Logger::println("ms");
  Logger::println();

  // センサー初期化
  if (!sensorReader.begin()) {
    Logger::println("ERROR: Sensor initialization failed!");
    while (1) { delay(100); }
  }

  // ステアリングコントローラー初期化
  steeringController.begin();

  // アクチュエーター初期化
  actuator.begin();

  Logger::println();
  Logger::println("System ready!");
  Logger::println();
}

// ============================================================================
// Loop関数
// ============================================================================
void loop() {
  static unsigned long lastMeasurement = 0;
  unsigned long currentTime = millis();

  // 指定した間隔で測定・制御（固定周期を維持）
  if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
    lastMeasurement += MEASUREMENT_INTERVAL;

    // =========================================================================
    // Phase 1: センサーデータ取得
    // =========================================================================
    sensorReader.readAll();
    const SensorData* sensorData = sensorReader.getAllData();

    // デバッグ: センサーデータ表示
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      Logger::printSensorData(
        SENSOR_CHANNELS[i],
        sensorData[i].distance,
        sensorData[i].valid
      );
      if (i < NUM_SENSORS - 1) {
        Logger::print(" | ");
      }
    }

    // =========================================================================
    // Phase 2: ステアリング角度計算（シンプル状態ベース）
    // =========================================================================
    float steering_angle = steeringController.calculate(sensorData);

    // デバッグ: モード、壁角度、ステアリング表示
    Logger::print(" | Mode:");
    Logger::print(steeringController.getModeName());
    Logger::print(" WA:");
    Logger::print(steeringController.getWallAngle(), 1);
    Logger::printSteering(steering_angle);

    // =========================================================================
    // Phase 3: アクチュエーター制御
    // =========================================================================
    // 緊急モード時は減速
    if (steeringController.getCurrentMode() == MODE_EMERGENCY) {
      actuator.setSteering(steering_angle);
      actuator.stop();
    } else {
      actuator.setSteering(steering_angle);
      actuator.setSpeed(BASE_SPEED_PULSE);
    }

    Logger::println("");
  }
}
