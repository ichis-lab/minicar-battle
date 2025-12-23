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

  // float出力（小数点以下桁数指定）
  static void print(float value, int decimals) {
    #if DEBUG_MODE
      Serial.print(value, decimals);
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
      Serial.print("S");
      Serial.print(channel);
      Serial.print(":");
      if (valid) {
        Serial.print(distance);
      } else {
        Serial.print("---");
      }
    #endif
  }

  // 壁検出状態の表示
  static void printWallStatus(bool left_valid, bool right_valid) {
    #if DEBUG_MODE
      Serial.print(" | W:");
      Serial.print(left_valid ? "L" : "-");
      Serial.print(right_valid ? "R" : "-");
    #endif
  }

  // 壁までの距離の表示
  static void printWallDistances(bool left_valid, float left_dist, bool right_valid, float right_dist) {
    #if DEBUG_MODE
      Serial.print(" D:");
      if (left_valid) {
        Serial.print((int)left_dist);
      } else {
        Serial.print("---");
      }
      Serial.print("/");
      if (right_valid) {
        Serial.print((int)right_dist);
      } else {
        Serial.print("---");
      }
    #endif
  }

  // ステアリング角度の表示
  static void printSteering(float angle) {
    #if DEBUG_MODE
      Serial.print(" St:");
      Serial.print(angle, 1);
    #endif
  }

  // アクチュエーター情報の表示
  static void printActuator(const char* name, uint16_t pulse_us) {
    #if DEBUG_MODE
      Serial.print(" [");
      Serial.print(name);
      Serial.print(":");
      Serial.print(pulse_us);
      Serial.print("]");
    #endif
  }
};

#endif // LOGGER_H
