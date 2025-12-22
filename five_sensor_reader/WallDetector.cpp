/*
 * WallDetector.cpp
 *
 * 壁検出クラス（実装）
 * 2つのセンサーペアから壁の直線を推定し、距離と角度を計算
 */

#include "WallDetector.h"
#include "SensorReader.h"

WallDetector::WallDetector() {
}

bool WallDetector::isValidReading(uint16_t distance) {
    return (distance >= MIN_VALID_DISTANCE && distance <= RELIABLE_RANGE);
}

bool WallDetector::calculateWall(uint16_t dist_far, uint16_t dist_near,
                                  float angle_far, float angle_near,
                                  float& out_distance, float& out_angle) {
    // 両方のセンサーが有効でなければ失敗
    if (!isValidReading(dist_far) || !isValidReading(dist_near)) {
        return false;
    }

    // センサー間の距離差が大きすぎる場合は信頼性が低い
    int16_t diff = abs((int16_t)dist_far - (int16_t)dist_near);
    if (diff > MAX_SENSOR_DIFF) {
        return false;
    }

    // 極座標から直交座標へ変換
    // センサー角度をラジアンに変換
    float rad_far = angle_far * PI / 180.0;
    float rad_near = angle_near * PI / 180.0;

    // 各センサーの検出点（車体座標系）
    float x_far = dist_far * sin(rad_far);
    float y_far = dist_far * cos(rad_far);
    float x_near = dist_near * sin(rad_near);
    float y_near = dist_near * cos(rad_near);

    // 2点を結ぶ直線の角度（壁の向き）
    float dx = x_far - x_near;
    float dy = y_far - y_near;

    if (abs(dy) < 0.001) {
        // ほぼ水平な壁（車体と平行）
        out_angle = 0.0;
    } else {
        out_angle = atan2(dx, dy) * 180.0 / PI;
    }

    // 壁までの距離（2点の中点からの垂直距離を近似）
    out_distance = (dist_far + dist_near) / 2.0;

    return true;
}

WallDetection WallDetector::detect(const SensorData* sensorData) {
    WallDetection result;
    result.left_valid = false;
    result.right_valid = false;
    result.left_distance = 0.0;
    result.right_distance = 0.0;
    result.left_angle = 0.0;
    result.right_angle = 0.0;

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
    result.left_valid = calculateWall(
        sensorData[0].distance, sensorData[1].distance,
        SENSOR_ANGLES[0], SENSOR_ANGLES[1],
        result.left_distance, result.left_angle
    );

    // 右壁の検出（センサー4と3を使用）
    result.right_valid = calculateWall(
        sensorData[4].distance, sensorData[3].distance,
        SENSOR_ANGLES[4], SENSOR_ANGLES[3],
        result.right_distance, result.right_angle
    );

    return result;
}
