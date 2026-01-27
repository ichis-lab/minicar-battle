/*
 * seven.ino
 *
 * 7つのVL53L1Xセンサーを使った Follow the Gap + Pure Pursuit制御
 *
 * 接続:
 * - TCA9548A I2Cマルチプレクサ (アドレス: 0x70)
 * - VL53L1Xセンサー → マルチプレクサのチャンネル0-6に接続
 * - サーボモーター → Pin 9
 * - ESC → Pin 10
 *
 * 使用方法:
 * 1. Arduino IDEでこのファイルを開く
 * 2. 必要なライブラリをインストール:
 *    - VL53L1X (Pololu)
 *    - Servo (Arduino標準ライブラリ)
 * 3. Config.hでRUN_MODEを設定
 *    - MODE_DEBUG: デバッグ専用（PWMなし、シリアルあり）
 *    - MODE_PRODUCTION: 本番走行（PWMあり、シリアルなし）
 *    - MODE_DEBUG_RUN: デバッグ走行（PWMあり、シリアルあり）
 * 4. Arduino Nano R4に書き込み
 */

#include "AcceleratorController.h"
#include "Actuator.h"
#include "Config.h"
#include "GapFinder.h"
#include "Logger.h"
#include "SensorReader.h"
#include "SteeringController.h"

// ============================================================================
// グローバルオブジェクト
// ============================================================================
SensorReader sensorReader;
GapFinder gapFinder;
SteeringController steeringController;
AcceleratorController acceleratorController;
Actuator actuator;

// ============================================================================
// Setup関数
// ============================================================================
void setup() {
    // ロガー初期化（115200bpsで詳細データ表示）
    Logger::begin(115200);

    Logger::println("==============================================");
    Logger::println("  VL53L1X Follow the Gap + Pure Pursuit");
    Logger::println("==============================================");
    Logger::print("Run Mode: ");
#if RUN_MODE == MODE_DEBUG
    Logger::println("DEBUG (No PWM, Serial ON)");
#elif RUN_MODE == MODE_PRODUCTION
    Logger::println("PRODUCTION (PWM ON, Serial OFF)");
#else
    Logger::println("DEBUG_RUN (PWM ON, Serial ON)");
#endif
    Logger::print("Measurement Interval: ");
    Logger::print(MEASUREMENT_INTERVAL);
    Logger::println("ms");
    Logger::print("Wheelbase: ");
    Logger::print(WHEELBASE_MM, 0);
    Logger::println("mm");
    Logger::println("Lookahead: Front sensor distance");
    Logger::println();

    // タイミング設定のサマリー表示
    Logger::printTimingConfig();
    Logger::println();

    // センサー初期化
    if (!sensorReader.begin()) {
        Logger::println("ERROR: Sensor initialization failed!");
        while (1) {
            delay(100);
        }
    }

    // ステアリングコントローラー初期化
    steeringController.begin();

    // アクセルコントローラー初期化
    acceleratorController.begin();

    // アクチュエーター初期化
    actuator.begin();

    // ESCアーミング待ち
    Logger::println("Waiting 3 seconds for ESC arming...");
    delay(3000);

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

#if ENABLE_BLUETOOTH_EMERGENCY
    // Bluetooth経由で's'のみ（単独）受信したら緊急停止
    if (Serial1.available()) {
        String received = "";
        while (Serial1.available()) {
            received += (char)Serial1.read();
        }
        if (received == "s") {
            actuator.setSteering(0.0);
            actuator.stop();
            while (1) {
                delay(1000);
            }  // 終了
        }
    }
#endif

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
            Logger::printSensorData(SENSOR_CHANNELS[i], sensorData[i].distance,
                                    sensorData[i].valid);
            if (i < NUM_SENSORS - 1) {
                Logger::print(" | ");
            }
        }

        // =========================================================================
        // Phase 2: 緊急停止チェック（前方障害物検出）
        // =========================================================================
        bool emergency_stop = false;
        if (sensorData[FRONT_SENSOR_INDEX].valid &&
            sensorData[FRONT_SENSOR_INDEX].distance < EMERGENCY_FRONT_THRESHOLD) {
            emergency_stop = true;
            Logger::print(" | EMERGENCY!");
        }

        // =========================================================================
        // Phase 3: ギャップ検出（最遠+隣接センサー方式）
        // =========================================================================
        GapResult gap = gapFinder.find(sensorData);

        // デバッグ: ギャップ検出結果表示（Ldは正面センサー距離）
        float front_distance = sensorData[FRONT_SENSOR_INDEX].valid
                               ? sensorData[FRONT_SENSOR_INDEX].distance
                               : 0.0f;
        Logger::printGapResult(gap.target_angle, front_distance);

        // =========================================================================
        // Phase 4: ステアリング角度計算（Pure Pursuit）
        // =========================================================================
        float steering_angle = steeringController.calculate(gap, sensorData);

        // デバッグ: ステアリング角度とブースト状態表示
        Logger::printSteering(steering_angle);
        Logger::printBoostStatus(gap.boost_left, gap.boost_right);

        // =========================================================================
        // Phase 5: アクチュエーター制御
        // =========================================================================
        if (emergency_stop) {
            // 緊急停止：中央ステアリング + 停止
            actuator.setSteering(0.0);
            actuator.stop();
        } else {
            // 通常走行：ステアリング角度に応じた可変速度
            actuator.setSteering(steering_angle);
            uint16_t variable_speed =
                acceleratorController.calculate(steering_angle, sensorData);
            actuator.setSpeed(variable_speed);
        }

        Logger::println("");
    }
}
