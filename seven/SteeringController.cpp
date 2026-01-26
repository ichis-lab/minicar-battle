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

#include "SensorReader.h"

SteeringController::SteeringController() {}

void SteeringController::begin() {
    // Pure Pursuitはステートレスなので初期化処理なし
}

float SteeringController::calculate(const GapResult& gap, const SensorData* sensorData) {
    // GapFinderの出力を目標点の極座標として解釈
    // α (alpha): 目標点への角度
    // Ld: ルックアヘッド距離 - 正面センサーの距離を使用

    float alpha_deg = gap.target_angle;

    // 動的オフセット: 目標角度が大きいほどオフセットを増やしてLdを短くする
    float dynamic_offset = LD_BASE_OFFSET_MM + LD_ANGLE_GAIN * fabs(alpha_deg);

    // 正面センサーの距離から動的オフセットを引いてルックアヘッド距離とする
    float Ld_mm = sensorData[FRONT_SENSOR_INDEX].valid
                  ? sensorData[FRONT_SENSOR_INDEX].distance - dynamic_offset
                  : 1000.0f;  // センサー無効時のフォールバック

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
