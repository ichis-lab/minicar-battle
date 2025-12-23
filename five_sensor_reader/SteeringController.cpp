/*
 * SteeringController.cpp
 *
 * ステアリング制御クラス（実装）
 * 角度ベース統一PID制御
 *
 * 設計思想:
 * - 全モードでPID入力を「角度（deg）」に統一
 * - モード切り替え時にPIDリセットしない（I項・D項が蓄積）
 * - 安全距離制約はPIDの外側で加算
 */

#include "SteeringController.h"
#include "Logger.h"

SteeringController::SteeringController() {
    _lastError = 0.0;
}

void SteeringController::begin() {
    // 統一PID初期化（角度ベース）
    _pid.begin(STEERING_KP, STEERING_KI, STEERING_KD);
    _pid.setOutputLimits(-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    _pid.setIntegralLimits(-STEERING_INTEGRAL_MAX, STEERING_INTEGRAL_MAX);
    _pid.setDeadband(STEERING_DEADBAND);
}

float SteeringController::calculate(const WallDetection& walls) {
    float error = 0.0;

    if (walls.left_valid && walls.right_valid) {
        // =================================================================
        // 両壁モード: 壁角度の平均 + 距離補正
        // =================================================================
        float center_angle = (walls.left_angle + walls.right_angle) / 2.0;
        float distance_correction = (walls.right_distance - walls.left_distance) * DISTANCE_TO_ANGLE_GAIN;
        error = center_angle + distance_correction;

    } else if (walls.left_valid) {
        // =================================================================
        // 左壁モード: 壁との角度
        // =================================================================
        error = walls.left_angle;

    } else if (walls.right_valid) {
        // =================================================================
        // 右壁モード: 壁との角度
        // =================================================================
        error = walls.right_angle;

    } else {
        // =================================================================
        // 壁なし: 直進
        // =================================================================
        _lastError = 0.0;
        return 0.0;
    }

    // エラー値を保存（デバッグ用）
    _lastError = error;

    // PID計算（モード切り替えでリセットしない）
    float steering = _pid.compute(0.0, error);

    // =========================================================================
    // 安全距離制約（PIDの外側で加算）
    // =========================================================================
    if (walls.left_valid && walls.left_distance < MIN_SAFE_DISTANCE) {
        float shortage = MIN_SAFE_DISTANCE - walls.left_distance;
        steering += shortage * DISTANCE_AVOID_GAIN;  // 右へ
    }
    if (walls.right_valid && walls.right_distance < MIN_SAFE_DISTANCE) {
        float shortage = MIN_SAFE_DISTANCE - walls.right_distance;
        steering -= shortage * DISTANCE_AVOID_GAIN;  // 左へ
    }

    // 最大操舵角でクランプ
    steering = constrain(steering, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    return steering;
}

void SteeringController::reset() {
    _pid.reset();
    _lastError = 0.0;
}
