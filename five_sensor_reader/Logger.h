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
    #if DEBUG_MODE
      Serial.begin(baud);
      while (!Serial) { delay(10); }  // シリアル接続待ち
    #endif
  }

  // 汎用プリント（改行なし）
  template<typename T>
  static void print(const T& value) {
    #if DEBUG_MODE
      Serial.print(value);
    #endif
  }

  // 汎用プリント（改行あり）
  template<typename T>
  static void println(const T& value) {
    #if DEBUG_MODE
      Serial.println(value);
    #endif
  }

  // 改行のみ（引数なし版）
  static void println() {
    #if DEBUG_MODE
      Serial.println();
    #endif
  }

  // センサーデータのコンパクト表示
  static void printSensorData(uint8_t channel, uint16_t distance, bool valid) {
    #if DEBUG_MODE
      Serial.print("Ch");
      Serial.print(channel);
      Serial.print(": ");
      if (valid) {
        Serial.print(distance);
        Serial.print("mm");
      } else {
        Serial.print("Out of range");
      }
    #endif
  }

  // 開放度データの表示
  static void printOpenness(float left, float right, float error) {
    #if DEBUG_MODE
      Serial.print(" | Open L:");
      Serial.print(left, 0);
      Serial.print(" R:");
      Serial.print(right, 0);
      Serial.print(" Err:");
      Serial.print(error, 0);
    #endif
  }

  // ステアリング角度の表示
  static void printSteering(float angle) {
    #if DEBUG_MODE
      Serial.print(" | Steer:");
      Serial.print(angle, 1);
      Serial.print("deg");
    #endif
  }

  // アクチュエーター情報の表示
  static void printActuator(const char* name, uint16_t pulse_us) {
    #if DEBUG_MODE
      Serial.print("  [");
      Serial.print(name);
      Serial.print(": ");
      Serial.print(pulse_us);
      Serial.print("us]");
    #endif
  }
};

#endif // LOGGER_H
