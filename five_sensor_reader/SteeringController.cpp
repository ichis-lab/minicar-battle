/*
 * SteeringController.cpp
 *
 * ステアリング制御クラス（実装）
 * 開放度ベースPID制御
 *
 * 左右の開放度バランスを制御目標とし、
 * 広いセクションでの不要な蛇行を防ぐ
 */

#include "SteeringController.h"
#include "SensorReader.h"

SteeringController::SteeringController()
    : opennessCalc(OPENNESS_WEIGHT_FAR, OPENNESS_WEIGHT_NEAR),
      pid(OPENNESS_PID_KP, OPENNESS_PID_KI, OPENNESS_PID_KD,
          OPENNESS_PID_INTEGRAL_LIMIT, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE) {
    // 開放度データの初期化
    lastOpennessData.left_openness = 0.0;
    lastOpennessData.right_openness = 0.0;
    lastOpennessData.error = 0.0;
    lastOpennessData.valid = false;
}

bool SteeringController::needsEmergencyAvoidance(uint16_t front_distance) {
    return (front_distance < EMERGENCY_FRONT_THRESHOLD);
}

float SteeringController::calculateEmergencySteering(const OpennessData& openness) {
    // 緊急回避: 開放度が大きい方へ最大舵角で回避
    if (openness.left_openness > openness.right_openness) {
        // 左が開けている → 左へ最大舵角
        return -MAX_STEERING_ANGLE;
    } else {
        // 右が開けている（または同等）→ 右へ最大舵角
        return MAX_STEERING_ANGLE;
    }
}

float SteeringController::calculate(const SensorData* sensorData) {
    if (sensorData == nullptr) {
        return 0.0;
    }

    // センサーデータから距離配列を作成
    uint16_t distances[5];
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        distances[i] = sensorData[i].valid ? sensorData[i].distance : SENSOR_ERROR_VALUE;
    }

    // 開放度を計算
    lastOpennessData = opennessCalc.calculate(distances);

    // 前方距離を取得（センサー2が前方）
    uint16_t front_distance = distances[2];

    float steering_angle = 0.0;

    // =========================================================================
    // 緊急回避チェック
    // =========================================================================
    if (sensorData[2].valid && needsEmergencyAvoidance(front_distance)) {
        // 緊急回避モード
        steering_angle = calculateEmergencySteering(lastOpennessData);
    }
    // =========================================================================
    // 通常制御：開放度ベースPID
    // =========================================================================
    else if (lastOpennessData.valid) {
        // PID制御で開放度の偏差を補正
        // error > 0: 右が開けている → 右へ（正のステアリング）
        // error < 0: 左が開けている → 左へ（負のステアリング）
        steering_angle = pid.calculate(lastOpennessData.error);
    }
    // =========================================================================
    // フォールバック：センサー無効時は直進
    // =========================================================================
    else {
        steering_angle = 0.0;
    }

    return steering_angle;
}

void SteeringController::reset() {
    pid.reset();
    lastOpennessData.left_openness = 0.0;
    lastOpennessData.right_openness = 0.0;
    lastOpennessData.error = 0.0;
    lastOpennessData.valid = false;
}

const OpennessData& SteeringController::getLastOpennessData() const {
    return lastOpennessData;
}
