/*
 * WallDetector.cpp
 *
 * 壁検出クラス（実装）
 * 2つのセンサーペアから壁の直線を推定し、垂直距離と角度を計算
 *
 * 計算方法:
 * 1. 各センサーの極座標（距離, 角度）を直交座標（x, y）に変換
 * 2. 2点を通る直線の方程式を求める
 * 3. 原点（車体位置）から直線への垂直距離を計算
 * 4. 直線の傾きから壁角度を計算
 */

#include "WallDetector.h"
#include "SensorReader.h"
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif

WallDetector::WallDetector() {
}

bool WallDetector::_isValidReading(uint16_t distance) {
    return (distance >= MIN_VALID_DISTANCE && distance <= RELIABLE_RANGE);
}

bool WallDetector::_calculateWall(uint16_t dist_far, uint16_t dist_near,
                                   float angle_far, float angle_near,
                                   float& out_distance, float& out_angle) {
    // 両方のセンサーが有効でなければ失敗
    if (!_isValidReading(dist_far) || !_isValidReading(dist_near)) {
        return false;
    }

    // センサー間の距離差が大きすぎる場合は信頼性が低い
    int16_t diff = std::abs((int16_t)dist_far - (int16_t)dist_near);
    if (diff > MAX_SENSOR_DIFF) {
        return false;
    }

    // 極座標から直交座標へ変換
    // センサー角度をラジアンに変換
    float rad_far = angle_far * PI / 180.0f;
    float rad_near = angle_near * PI / 180.0f;

    // 各センサーの検出点（車体座標系）
    // x: 左が負、右が正
    // y: 前方が正
    float x1 = dist_far * std::sin(rad_far);
    float y1 = dist_far * std::cos(rad_far);
    float x2 = dist_near * std::sin(rad_near);
    float y2 = dist_near * std::cos(rad_near);

    // 2点間の差分
    float dx = x2 - x1;
    float dy = y2 - y1;

    // 2点間の距離
    float line_length = std::sqrt(dx * dx + dy * dy);
    if (line_length < 0.001f) {
        // 2点がほぼ同じ位置（計算不能）
        return false;
    }

    // =========================================================================
    // 壁までの垂直距離を計算
    // 2点 P1(x1,y1), P2(x2,y2) を通る直線と原点(0,0)の距離
    // 公式: d = |x1*y2 - x2*y1| / sqrt((x2-x1)^2 + (y2-y1)^2)
    // =========================================================================
    float cross_product = x1 * y2 - x2 * y1;
    out_distance = std::abs(cross_product) / line_length;

    // =========================================================================
    // 壁の角度を計算
    // 直線が車体正面方向（y軸）となす角度
    // angle > 0: 前方で壁に近づいている
    // angle < 0: 前方で壁から離れている
    // angle = 0: 壁と平行
    // =========================================================================
    if (std::abs(dy) < 0.001f) {
        // ほぼ水平な壁（車体と平行）
        out_angle = 0.0f;
    } else {
        out_angle = std::atan2(dx, dy) * 180.0f / PI;
    }

    return true;
}

WallDetection WallDetector::detect(const SensorData* sensorData) {
    WallDetection result;
    result.left_valid = false;
    result.right_valid = false;
    result.left_distance = 0.0f;
    result.right_distance = 0.0f;
    result.left_angle = 0.0f;
    result.right_angle = 0.0f;

    if (sensorData == nullptr) {
        return result;
    }

    // センサー配置:
    // Index 0: -70° (左側方)
    // Index 1: -20° (左前方)
    // Index 2:   0° (前方)
    // Index 3: +20° (右前方)
    // Index 4: +70° (右側方)

    // 左壁の検出（センサー0と1を使用）
    result.left_valid = _calculateWall(
        sensorData[0].distance, sensorData[1].distance,
        SENSOR_ANGLES[0], SENSOR_ANGLES[1],
        result.left_distance, result.left_angle
    );

    // 右壁の検出（センサー4と3を使用）
    result.right_valid = _calculateWall(
        sensorData[4].distance, sensorData[3].distance,
        SENSOR_ANGLES[4], SENSOR_ANGLES[3],
        result.right_distance, result.right_angle
    );

    return result;
}
