/*
 * SteeringController.cpp
 *
 * ステアリング制御クラス（実装）
 * PID制御による壁追従制御
 *
 * 制御モード:
 * - 両壁モード: 距離PIDで左右中央を維持
 * - 片壁モード: 角度PIDで壁と平行を維持 + 安全距離制約
 * - 壁なし: 直進
 */

#include "SteeringController.h"
#include "Logger.h"

SteeringController::SteeringController() {
    _currentMode = MODE_NO_WALLS;
    _previousMode = MODE_NO_WALLS;
}

void SteeringController::begin() {
    // =========================================================================
    // 中央走行PID初期化（両壁モード：距離制御）
    // 目標: 左右壁からの距離差 = 0
    // =========================================================================
    _centeringPID.begin(STEERING_KP, STEERING_KI, STEERING_KD);
    _centeringPID.setOutputLimits(-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    _centeringPID.setIntegralLimits(-STEERING_INTEGRAL_MAX, STEERING_INTEGRAL_MAX);
    _centeringPID.setDeadband(10.0);  // 10mm不感帯

    // =========================================================================
    // 角度PID初期化（片壁モード：角度制御）
    // 目標: 壁角度 = 0°（壁と平行）
    // =========================================================================
    _anglePID.begin(ANGLE_KP, ANGLE_KI, ANGLE_KD);
    _anglePID.setOutputLimits(-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    _anglePID.setIntegralLimits(-STEERING_INTEGRAL_MAX, STEERING_INTEGRAL_MAX);
    _anglePID.setDeadband(1.0);  // 1°不感帯
}

float SteeringController::calculate(const WallDetection& walls) {
    float steering_angle = 0.0;

    // モード判定
    _previousMode = _currentMode;

    if (walls.left_valid && walls.right_valid) {
        _currentMode = MODE_BOTH_WALLS;
    } else if (walls.left_valid) {
        _currentMode = MODE_LEFT_WALL;
    } else if (walls.right_valid) {
        _currentMode = MODE_RIGHT_WALL;
    } else {
        _currentMode = MODE_NO_WALLS;
    }

    // モード変更時にPIDをリセット
    if (_currentMode != _previousMode) {
        _centeringPID.reset();
        _anglePID.reset();
    }

    // モードに応じた制御
    switch (_currentMode) {
        case MODE_BOTH_WALLS: {
            // =================================================================
            // 両壁モード: 距離PIDで中央を維持
            // 目標: 左右壁からの距離差 = 0
            // 誤差 = 右壁距離 - 左壁距離（正なら左寄り → 右へステア）
            // =================================================================
            float error = walls.right_distance - walls.left_distance;
            float setpoint = TARGET_CENTER_OFFSET;  // 通常は0
            steering_angle = _centeringPID.compute(setpoint, -error);
            break;
        }

        case MODE_LEFT_WALL: {
            // =================================================================
            // 左壁モード: 角度PIDで壁と平行を維持
            // 目標: 壁角度 = 0°
            // 壁角度 > 0: 壁に向かっている → 右へステア（正）
            // 壁角度 < 0: 壁から離れている → 左へステア（負）
            // =================================================================
            steering_angle = -_anglePID.compute(0.0, walls.left_angle);

            // 安全距離制約: 壁に近すぎる場合は離れる方向に補正
            if (walls.left_distance < MIN_SAFE_DISTANCE) {
                float shortage = MIN_SAFE_DISTANCE - walls.left_distance;
                steering_angle += shortage * DISTANCE_AVOID_GAIN;  // 右へ（正）
            }
            break;
        }

        case MODE_RIGHT_WALL: {
            // =================================================================
            // 右壁モード: 角度PIDで壁と平行を維持
            // 目標: 壁角度 = 0°
            // 壁角度 > 0: 壁に向かっている → 左へステア（負）
            // 壁角度 < 0: 壁から離れている → 右へステア（正）
            // =================================================================
            steering_angle = -_anglePID.compute(0.0, walls.right_angle);

            // 安全距離制約: 壁に近すぎる場合は離れる方向に補正
            if (walls.right_distance < MIN_SAFE_DISTANCE) {
                float shortage = MIN_SAFE_DISTANCE - walls.right_distance;
                steering_angle -= shortage * DISTANCE_AVOID_GAIN;  // 左へ（負）
            }
            break;
        }

        case MODE_NO_WALLS:
        default:
            // =================================================================
            // 壁なし: 直進維持
            // =================================================================
            steering_angle = 0.0;
            break;
    }

    // 最大操舵角でクランプ
    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    return steering_angle;
}

void SteeringController::reset() {
    _centeringPID.reset();
    _anglePID.reset();
    _currentMode = MODE_NO_WALLS;
    _previousMode = MODE_NO_WALLS;
}

void SteeringController::printDebugInfo() const {
    Logger::print(" Mode:");
    switch (_currentMode) {
        case MODE_BOTH_WALLS: Logger::print("BOTH"); break;
        case MODE_LEFT_WALL:  Logger::print("LEFT"); break;
        case MODE_RIGHT_WALL: Logger::print("RIGHT"); break;
        case MODE_NO_WALLS:   Logger::print("NONE"); break;
    }
}
