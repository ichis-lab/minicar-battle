/*
 * SteeringController.cpp
 *
 * ステアリング制御クラス（実装）
 * Follow the Gap + Pure Pursuit制御
 *
 * 設計思想:
 * - GapFinderが検出した目標方向をPure Pursuit公式に適用
 * - 公式: steering = atan2(2 × L × sin(α), Ld)
 *   L: ホイールベース（mm）
 *   α: 目標点への角度（ラジアン）
 *   Ld: ルックアヘッド距離（mm）- 正面センサーの距離を使用
 */

#include "SteeringController.h"

#include <math.h>

SteeringController::SteeringController() {}

void SteeringController::begin() {
    // Pure Pursuitはステートレスなので初期化処理なし
}

float SteeringController::calculate(const GapResult& gap, const SensorData* sensorData) {
    // GapFinderの出力を目標点の極座標として解釈
    // α (alpha): 目標点への角度
    // Ld: ルックアヘッド距離 - 正面センサーの距離を使用

    float alpha_deg = gap.target_angle;

    // 正面センサーの距離に応じてオフセットを動的に計算し、ルックアヘッド距離を算出
    float front_dist = sensorData[FRONT_SENSOR_INDEX].valid
                       ? (float)sensorData[FRONT_SENSOR_INDEX].distance
                       : 2000.0f;  // センサー無効時のフォールバック

    // 線形補間: 正面が近い→オフセット大（応答鋭い）、遠い→オフセット小（応答穏やか）
    float offset = (float)map(
        (long)(front_dist),
        (long)LOOKAHEAD_NEAR_DIST, (long)LOOKAHEAD_FAR_DIST,
        (long)LOOKAHEAD_OFFSET_NEAR, (long)LOOKAHEAD_OFFSET_FAR);
    offset = constrain(offset, LOOKAHEAD_OFFSET_FAR, LOOKAHEAD_OFFSET_NEAR);

    float Ld_mm = front_dist - offset;

    // ゼロ除算防止
    if (Ld_mm < 50.0f) Ld_mm = 50.0f;

    // 角度をラジアンに変換
    float alpha_rad = alpha_deg * DEG_TO_RAD;

    // Pure Pursuit公式: δ = atan2(2 × L × sin(α), Ld)
    float steering_rad = atan2(2.0f * WHEELBASE_MM * sin(alpha_rad), Ld_mm);

    // 度に変換
    float steering_deg = steering_rad * RAD_TO_DEG;

    // 最大操舵角でクランプ
    steering_deg = constrain(steering_deg, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    return steering_deg;
}
