/*
 * SteeringController.cpp
 *
 * シンプル状態ベース制御（実装）
 *
 * 優先度:
 * 1. 側壁接近回避（常時チェック）
 * 2. 緊急回避（正面が非常に近い）
 * 3. コーナリング（正面に壁）
 * 4. 直進 + 右壁追従
 */

#include "SteeringController.h"
#include "Logger.h"

SteeringController::SteeringController() {
    _currentMode = MODE_STRAIGHT;
    _lastSteering = 0.0;
}

void SteeringController::begin() {
    _currentMode = MODE_STRAIGHT;
    _lastSteering = 0.0;
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
        _lastSteering = EMERGENCY_AVOID_ANGLE;  // 右へ
        return _lastSteering;
    }
    if (right_far < MIN_SIDE_DISTANCE) {
        _currentMode = MODE_SIDE_AVOID;
        _lastSteering = -EMERGENCY_AVOID_ANGLE;  // 左へ
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
            steering = -CORNER_ANGLE;  // 左へ
        } else {
            steering = CORNER_ANGLE;   // 右へ
        }
        _lastSteering = steering;
        return steering;
    }

    // ========================================
    // 優先度4: 直進 + 右壁追従
    // ========================================
    _currentMode = MODE_STRAIGHT;

    // 右壁との距離で微調整
    if (right_far < TARGET_WALL_DISTANCE - WALL_TOLERANCE) {
        // 右壁に近すぎる → 左へ
        steering = -WALL_AVOID_ANGLE;
    } else if (right_far > TARGET_WALL_DISTANCE + WALL_TOLERANCE) {
        // 右壁から遠すぎる → 右へ
        steering = WALL_APPROACH_ANGLE;
    } else {
        // 適正距離 → 直進
        steering = 0.0;
    }

    _lastSteering = steering;
    return steering;
}

const char* SteeringController::getModeName() const {
    switch (_currentMode) {
        case MODE_STRAIGHT:   return "STRAIGHT";
        case MODE_CORNER:     return "CORNER";
        case MODE_EMERGENCY:  return "EMERGENCY";
        case MODE_SIDE_AVOID: return "SIDE_AVOID";
        default:              return "UNKNOWN";
    }
}
