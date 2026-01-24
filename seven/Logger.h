/*
 * Logger.h
 *
 * デバッグ出力管理（ヘッダーオンリー）
 * DEBUG_MODEに応じてシリアル出力を制御
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#include "Config.h"

// ============================================================================
// デバッグモードに応じたシリアル出力制御
// ============================================================================
class Logger {
   public:
    // シリアル初期化
    static void begin(unsigned long baud = 9600) {
#if ENABLE_SERIAL
        Serial.begin(baud);
        while (!Serial) {
            delay(10);
        }  // シリアル接続待ち
#endif
#if ENABLE_BLUETOOTH_LOGGING || ENABLE_BLUETOOTH_EMERGENCY
        Serial1.begin(BLUETOOTH_BAUD);
#endif
    }

    // 汎用プリント（改行なし）
    template <typename T>
    static void print(const T& value) {
#if ENABLE_SERIAL
        Serial.print(value);
#endif
#if ENABLE_BLUETOOTH_LOGGING
        Serial1.print(value);
#endif
    }

    // float出力（小数点以下桁数指定）
    static void print(float value, int decimals) {
#if ENABLE_SERIAL
        Serial.print(value, decimals);
#endif
#if ENABLE_BLUETOOTH_LOGGING
        Serial1.print(value, decimals);
#endif
    }

    // 汎用プリント（改行あり）
    template <typename T>
    static void println(const T& value) {
#if ENABLE_SERIAL
        Serial.println(value);
#endif
#if ENABLE_BLUETOOTH_LOGGING
        Serial1.println(value);
#endif
    }

    // 改行のみ（引数なし版）
    static void println() {
#if ENABLE_SERIAL
        Serial.println();
#endif
#if ENABLE_BLUETOOTH_LOGGING
        Serial1.println();
#endif
    }

    // センサーデータのコンパクト表示
    static void printSensorData(uint8_t channel, uint16_t distance,
                                bool valid) {
        print("S");
        print(channel);
        print(":");
        if (valid) {
            print(distance);
        } else {
            print("---");
        }
    }

    // ギャップ検出結果の表示
    static void printGapResult(float targetAngle, float targetDistance) {
        print(" | T:");
        print(targetAngle, 1);
        print("° Ld:");
        print((int)targetDistance);
    }

    // ステアリング角度の表示
    static void printSteering(float angle) {
        print(" St:");
        print(angle, 1);
    }

    // ブースト適用状態の表示
    static void printBoostStatus(bool boostLeft, bool boostRight) {
        if (boostLeft || boostRight) {
            print(" B:");
            if (boostLeft) print("L");
            if (boostRight) print("R");
        }
    }

    // アクチュエーター情報の表示
    static void printActuator(const char* name, uint16_t pulse_us) {
        print(" [");
        print(name);
        print(":");
        print(pulse_us);
        print("]");
    }

    // タイミング設定のサマリー表示
    static void printTimingConfig() {
        println("--- Timing Configuration ---");
        print("  L1X Timing Budget: ");
        print(L1X_TIMING_BUDGET_US);
        println(" us");
        print("  L1X Inter-Measurement: ");
        print(L1X_INTER_MEASUREMENT_MS);
        println(" ms");
        print("  Loop Interval: ");
        print(MEASUREMENT_INTERVAL);
        println(" ms");
        print("  Sensor Count: ");
        print(NUM_SENSORS);
        println(" (continuous mode, ~5ms/sensor)");
        println("----------------------------");
    }
};

#endif  // LOGGER_H
