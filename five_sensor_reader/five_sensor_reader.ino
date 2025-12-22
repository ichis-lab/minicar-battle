/*
 * five_sensor_reader.ino
 *
 * 5つのVL53L0Xセンサーを使った開放度ベースPID制御システム
 * オブジェクト指向設計版
 *
 * 接続:
 * - TCA9548A I2Cマルチプレクサ (アドレス: 0x70)
 * - VL53L0Xセンサー → マルチプレクサのチャンネル0-4に接続
 * - サーボモーター → Pin 9
 * - ESC → Pin 10
 *
 * 使用方法:
 * 1. Arduino IDEでこのファイルを開く
 * 2. 必要なライブラリをインストール:
 *    - Adafruit_VL53L0X
 *    - Servo (Arduino標準ライブラリ)
 * 3. Config.hでDEBUG_MODEを設定
 *    - true: デバッグ（PWMなし、シリアル出力あり）
 *    - false: 実機（PWMあり、シリアル出力なし）
 * 4. Arduino Nano R4に書き込み
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
  // ロガー初期化
  Logger::begin(9600);

  Logger::println("==========================================");
  Logger::println("  Openness-based PID Control System");
  Logger::println("==========================================");
  Logger::print("Debug Mode: ");
  Logger::println(DEBUG_MODE ? "ON (No PWM)" : "OFF (PWM Active)");
  Logger::println();

  // センサー初期化
  if (!sensorReader.begin()) {
    Logger::println("ERROR: Sensor initialization failed!");
    while (1) { delay(100); }  // 無限ループで停止
  }

  // アクチュエーター初期化
  actuator.begin();

  Logger::println("System ready!");
  Logger::println();
}

// ============================================================================
// Loop関数
// ============================================================================
void loop() {
  static unsigned long lastMeasurement = 0;
  unsigned long currentTime = millis();

  // 指定した間隔で測定・制御
  if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
    lastMeasurement = currentTime;

    // フェーズ1: センサーデータ取得
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
        Logger::print("  |  ");
      }
    }

    // フェーズ2: 緊急停止チェック（前方障害物検出）
    bool emergency_stop = false;
    if (sensorData[2].valid && sensorData[2].distance < EMERGENCY_FRONT_THRESHOLD) {
      emergency_stop = true;
      Logger::print(" | EMERGENCY STOP! Front:");
      Logger::print(sensorData[2].distance);
      Logger::print("mm");
    }

    // フェーズ3: ステアリング角度計算（開放度ベースPID）
    float steering_angle = steeringController.calculate(sensorData);

    // デバッグ: 開放度データ表示
    const OpennessData& openness = steeringController.getLastOpennessData();
    Logger::printOpenness(openness.left_openness, openness.right_openness, openness.error);

    // デバッグ: ステアリング表示
    Logger::printSteering(steering_angle);

    // フェーズ4: アクチュエーター制御
    if (emergency_stop) {
      // 緊急停止：中央ステアリング + 停止
      actuator.setSteering(0.0);
      actuator.stop();
    } else {
      // 通常走行
      actuator.setSteering(steering_angle);
      actuator.setSpeed(BASE_SPEED_PULSE);
    }

    Logger::println("");
  }
}
