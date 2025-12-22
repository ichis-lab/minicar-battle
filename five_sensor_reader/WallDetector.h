/*
 * WallDetector.h
 *
 * 壁検出クラス（宣言）
 * センサーデータから左右の壁を検出し、距離と角度を計算
 */

#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H

#include <Arduino.h>
#include "Config.h"

// 前方宣言
struct SensorData;

// 壁検出結果
struct WallDetection {
    bool left_valid;        // 左壁検出有効
    bool right_valid;       // 右壁検出有効
    float left_distance;    // 左壁までの距離（mm）
    float right_distance;   // 右壁までの距離（mm）
    float left_angle;       // 左壁の角度（度）
    float right_angle;      // 右壁の角度（度）
};

class WallDetector {
private:
    // センサーペアから壁の距離と角度を計算
    bool _calculateWall(uint16_t dist_far, uint16_t dist_near,
                        float angle_far, float angle_near,
                        float& out_distance, float& out_angle);

    // センサー値の妥当性チェック
    bool _isValidReading(uint16_t distance);

public:
    WallDetector();

    // 壁検出を実行
    WallDetection detect(const SensorData* sensorData);
};

#endif // WALL_DETECTOR_H
