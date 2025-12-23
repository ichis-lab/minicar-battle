/*
 * SteeringController.cpp
 *
 * シンプル状態ベース制御（実装）
 *
 * 優先度:
 * 1. 側壁接近回避（常時チェック）
 * 2. 緊急回避（正面が非常に近い）
 * 3. コーナリング（正面に壁）
 * 4. 直進 + 右壁追従（角度補正付き）
 */

#include "SteeringController.h"
#include "Logger.h"

SteeringController::SteeringController() {
    _currentMode = MODE_STRAIGHT;
    _lastSteering = 0.0;
    _wallAngle = 0.0;
}

void SteeringController::begin() {
    _currentMode = MODE_STRAIGHT;
    _lastSteering = 0.0;
    _wallAngle = 0.0;
}

float SteeringController::calculate(const SensorData* sensors) {
    if (sensors == nullptr) {
        return 0.0;
    }

    // センサー値を取得（無効な場合は最大距離として扱う）
    uint16_t left_far = sensors[0].valid ? sensors[0].distance : RELIABLE_RANGE;
    uint16_t left_near = sensors[1].valid ? sensors[1].distance : RELIABLE_RANGE;
    uint16_t front = sensors[2].valid ? sensors[2].distance : RELIABLE_RANGE;
    uint16_t right_near = sensors[3].valid ? sensors[3].distance : RELIABLE_RANGE;
    uint16_t right_far = sensors[4].valid ? sensors[4].distance : RELIABLE_RANGE;

    float steering = 0.0;

    // ========================================
    // 優先度1: 側壁接近回避（常時チェック）
    // ========================================
    if (left_far < MIN_SIDE_DISTANCE) {
        _currentMode = MODE_SIDE_AVOID;
        _lastSteering = MAX_STEERING_ANGLE * 0.7;  // 右へ
        return _lastSteering;
    }
    if (right_far < MIN_SIDE_DISTANCE) {
        _currentMode = MODE_SIDE_AVOID;
        _lastSteering = -MAX_STEERING_ANGLE * 0.7;  // 左へ
        return _lastSteering;
    }

    // ========================================
    // 優先度2: 緊急回避（正面が非常に近い）
    // ========================================
    if (front < EMERGENCY_THRESHOLD) {
        _currentMode = MODE_EMERGENCY;
        uint16_t left_space = min(left_far, left_near);
        uint16_t right_space = min(right_near, right_far);

        if (left_space > right_space) {
            steering = -MAX_STEERING_ANGLE;  // 左へ
        } else {
            steering = MAX_STEERING_ANGLE;   // 右へ
        }
        _lastSteering = steering;
        return steering;
    }

    // ========================================
    // 優先度3: コーナリング（正面に壁）
    // ========================================
    if (front < CORNER_THRESHOLD) {
        _currentMode = MODE_CORNER;
        uint16_t left_space = min(left_far, left_near);
        uint16_t right_space = min(right_near, right_far);

        if (left_space > right_space) {
            steering = -MAX_STEERING_ANGLE * 0.75;  // 左へ
        } else {
            steering = MAX_STEERING_ANGLE * 0.75;   // 右へ
        }
        _lastSteering = steering;
        return steering;
    }

    // ========================================
    // 優先度4: 直進 + 右壁追従
    // ========================================
    _currentMode = MODE_STRAIGHT;

    // S3, S4の距離差から壁角度を推定
    // diff > 0: S4が遠い = 壁から離れている方向
    // diff < 0: S4が近い = 壁に向かっている方向
    int wallDiff = (int)right_far - (int)right_near;
    _wallAngle = -wallDiff * WALL_STEERING_GAIN;

    // 目標距離との差も加味
    int distError = (int)right_far - (int)TARGET_WALL_DISTANCE;

    // ステアリング = 壁角度補正 + 距離補正
    // 壁に向かっている → 左へ、離れている → 右へ
    // 近すぎる → 左へ、遠すぎる → 右へ
    steering = _wallAngle + distError * WALL_STEERING_GAIN * 0.5;

    // 最大操舵角でクランプ
    steering = constrain(steering, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    _lastSteering = steering;
    return steering;
}

const char* SteeringController::getModeName() const {
    switch (_currentMode) {
        case MODE_STRAIGHT:   return "ST";
        case MODE_CORNER:     return "CN";
        case MODE_EMERGENCY:  return "EM";
        case MODE_SIDE_AVOID: return "SA";
        default:              return "??";
    }
}
