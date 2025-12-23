/*
 * five_sensor_reader.ino
 *
 * 5つのVL53L1Xセンサーを使った角度ベース統一PID制御
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
#include "WallDetector.h"
#include "SteeringController.h"
#include "Actuator.h"

// ============================================================================
// グローバルオブジェクト
// ============================================================================
SensorReader sensorReader;
WallDetector wallDetector;
SteeringController steeringController;
Actuator actuator;

// ============================================================================
// Setup関数
// ============================================================================
void setup() {
  // ロガー初期化（115200bpsで詳細データ表示）
  Logger::begin(115200);

  Logger::println("==========================================");
  Logger::println("  VL53L1X Angle-Based PID Control");
  Logger::println("==========================================");
  Logger::print("Debug Mode: ");
  Logger::println(DEBUG_MODE ? "ON (No PWM)" : "OFF (PWM Active)");
  Logger::print("Measurement Interval: ");
  Logger::print(MEASUREMENT_INTERVAL);
  Logger::println("ms");
  Logger::print("PID Gains: Kp=");
  Logger::print(STEERING_KP);
  Logger::print(" Ki=");
  Logger::print(STEERING_KI);
  Logger::print(" Kd=");
  Logger::println(STEERING_KD);
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
    // Phase 2: 緊急停止チェック（前方障害物検出）
    // =========================================================================
    bool emergency_stop = false;
    if (sensorData[2].valid && sensorData[2].distance < EMERGENCY_FRONT_THRESHOLD) {
      emergency_stop = true;
      Logger::print(" | EMERGENCY!");
    }

    // =========================================================================
    // Phase 3: 壁検出
    // =========================================================================
    WallDetection walls = wallDetector.detect(sensorData);

    // デバッグ: 壁検出結果表示（状態、距離、角度）
    Logger::printWallStatus(walls.left_valid, walls.right_valid);
    Logger::printWallDistances(walls.left_valid, walls.left_distance,
                               walls.right_valid, walls.right_distance);
    Logger::printWallAngles(walls.left_valid, walls.left_angle,
                            walls.right_valid, walls.right_angle);

    // =========================================================================
    // Phase 4: ステアリング角度計算（角度ベース統一PID）
    // =========================================================================
    float steering_angle = steeringController.calculate(walls);

    // デバッグ: エラー値とステアリング表示
    Logger::printError(steeringController.getLastError());
    Logger::printSteering(steering_angle);

    // =========================================================================
    // Phase 5: アクチュエーター制御
    // =========================================================================
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
