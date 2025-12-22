/*
 * Arduino.h - テスト用スタブ
 *
 * Arduino固有の関数・型をC++標準で再実装
 */

#ifndef ARDUINO_H_STUB
#define ARDUINO_H_STUB

#include <cstdint>
#include <cmath>
#include <algorithm>

// Arduino標準の型定義
typedef uint8_t byte;

// Arduino定数
#ifndef PI
#define PI 3.14159265358979323846
#endif

// Arduino標準関数のスタブ

// constrain: 値を範囲内に制限
template<typename T>
T constrain(T x, T minVal, T maxVal) {
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

// map: 値を別の範囲にマッピング
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// abs: 絶対値（整数用）
// 注: C++標準のabs()と衝突しないように注意
// Arduino版はマクロだがここでは使わない（cmath版を使う）

// millis: 経過時間（テスト用にグローバル変数で制御）
namespace ArduinoMock {
    extern unsigned long mock_millis;

    inline void setMillis(unsigned long ms) {
        mock_millis = ms;
    }

    inline void advanceMillis(unsigned long ms) {
        mock_millis += ms;
    }

    inline void resetMillis() {
        mock_millis = 0;
    }
}

inline unsigned long millis() {
    return ArduinoMock::mock_millis;
}

#endif // ARDUINO_H_STUB
