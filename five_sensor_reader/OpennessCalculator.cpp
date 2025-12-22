/*
 * OpennessCalculator.cpp
 *
 * 開放度計算クラス（実装）
 * Openness Calculator Implementation
 *
 * 左右のセンサーから開放度を計算し、制御の偏差を算出
 */

#include "OpennessCalculator.h"

OpennessCalculator::OpennessCalculator(float w_far, float w_near)
    : weight_far(w_far), weight_near(w_near) {
}

bool OpennessCalculator::isValidReading(uint16_t distance) {
    // 有効範囲: MIN_VALID_DISTANCE ～ RELIABLE_RANGE
    return (distance >= MIN_VALID_DISTANCE && distance <= RELIABLE_RANGE);
}

float OpennessCalculator::calculateSideOpenness(uint16_t dist_far, uint16_t dist_near) {
    // 無効な値は最大距離として扱う（開放されていると見なす）
    float far_value = isValidReading(dist_far) ? (float)dist_far : (float)RELIABLE_RANGE;
    float near_value = isValidReading(dist_near) ? (float)dist_near : (float)RELIABLE_RANGE;

    // 重み付き平均で開放度を計算
    return weight_far * far_value + weight_near * near_value;
}

OpennessData OpennessCalculator::calculate(const uint16_t distances[5]) {
    OpennessData result;

    // センサー配置:
    // Index 0: -70° (左側方)
    // Index 1: -20° (左前方)
    // Index 2:   0° (前方) - 開放度計算には使用しない
    // Index 3: +20° (右前方)
    // Index 4: +70° (右側方)

    // 左側開放度 (センサー0, 1)
    result.left_openness = calculateSideOpenness(distances[0], distances[1]);

    // 右側開放度 (センサー4, 3)
    result.right_openness = calculateSideOpenness(distances[4], distances[3]);

    // 偏差 = 右開放度 - 左開放度
    // error > 0: 右が開けている → 右へステアリング
    // error < 0: 左が開けている → 左へステアリング
    // error ≈ 0: 均等 → 直進
    result.error = result.right_openness - result.left_openness;

    // 最低1つのセンサーが有効であれば計算は有効
    bool any_valid = isValidReading(distances[0]) ||
                     isValidReading(distances[1]) ||
                     isValidReading(distances[3]) ||
                     isValidReading(distances[4]);
    result.valid = any_valid;

    return result;
}
