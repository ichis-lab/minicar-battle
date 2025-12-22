/*
 * OpennessCalculator.h
 *
 * 開放度計算クラス（宣言）
 * Openness Calculator
 *
 * センサーデータから左右の開放度を計算
 * Calculates left/right openness from sensor data
 */

#ifndef OPENNESS_CALCULATOR_H
#define OPENNESS_CALCULATOR_H

#include <Arduino.h>
#include "Config.h"

/**
 * 開放度計算結果
 * Openness calculation result
 */
struct OpennessData {
    float left_openness;    // 左側開放度 / Left side openness
    float right_openness;   // 右側開放度 / Right side openness
    float error;            // 偏差 (right - left) / Error
    bool valid;             // 計算が有効か / Calculation validity
};

/**
 * 開放度計算クラス
 * Openness Calculator
 */
class OpennessCalculator {
private:
    float weight_far;   // 70°センサーの重み / Weight for 70° sensors
    float weight_near;  // 20°センサーの重み / Weight for 20° sensors

    /**
     * 片側の開放度を計算
     * Calculate openness for one side
     */
    float calculateSideOpenness(uint16_t dist_far, uint16_t dist_near);

    /**
     * センサー値の妥当性チェック
     * Validate sensor reading
     */
    bool isValidReading(uint16_t distance);

public:
    /**
     * コンストラクタ
     * @param w_far 70°センサーの重み / Weight for 70° sensors
     * @param w_near 20°センサーの重み / Weight for 20° sensors
     */
    OpennessCalculator(float w_far, float w_near);

    /**
     * 開放度を計算
     * Calculate openness
     * @param distances センサー距離配列[5] / Sensor distance array[5]
     * @return 開放度データ / Openness data
     */
    OpennessData calculate(const uint16_t distances[5]);
};

#endif // OPENNESS_CALCULATOR_H
