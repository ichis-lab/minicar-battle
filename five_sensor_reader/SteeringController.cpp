/*
 * SteeringController.cpp
 *
 * ステアリング制御クラス（実装）
 * PID制御による壁追従制御
 */

#include "SteeringController.h"
#include "SensorReader.h"
#include "Logger.h"

SteeringController::SteeringController() {
    currentMode = MODE_NO_WALLS;
    previousMode = MODE_NO_WALLS;
}

void SteeringController::begin() {
    // 中央走行PID初期化 / Initialize centering PID
    centeringPID.begin(STEERING_KP, STEERING_KI, STEERING_KD);
    centeringPID.setOutputLimits(-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    centeringPID.setIntegralLimits(-STEERING_INTEGRAL_MAX, STEERING_INTEGRAL_MAX);
    centeringPID.setDeadband(10.0);  // 10mm不感帯 / 10mm deadband

    // 壁追従PID初期化 / Initialize wall following PID
    // 壁追従時はゲインを少し高めに設定
    wallFollowPID.begin(STEERING_KP * 1.2, STEERING_KI, STEERING_KD * 1.5);
    wallFollowPID.setOutputLimits(-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    wallFollowPID.setIntegralLimits(-STEERING_INTEGRAL_MAX, STEERING_INTEGRAL_MAX);
    wallFollowPID.setDeadband(5.0);
}

float SteeringController::calculate(const WallDetection& walls, const SensorData* sensorData) {
    float steering_angle = 0.0;

    // モード判定 / Mode determination
    previousMode = currentMode;

    if (walls.left_valid && walls.right_valid) {
        currentMode = MODE_BOTH_WALLS;
    } else if (walls.left_valid) {
        currentMode = MODE_LEFT_WALL;
    } else if (walls.right_valid) {
        currentMode = MODE_RIGHT_WALL;
    } else {
        currentMode = MODE_NO_WALLS;
    }

    // モード変更時にPIDをリセット / Reset PID on mode change
    if (currentMode != previousMode) {
        centeringPID.reset();
        wallFollowPID.reset();
    }

    // モードに応じた制御 / Mode-specific control
    switch (currentMode) {
        case MODE_BOTH_WALLS: {
            // 目標: 左右壁の中央 / Target: center between walls
            // 誤差 = 右壁距離 - 左壁距離（正なら左寄り）
            // Error = right_distance - left_distance (positive = too left)
            float error = walls.right_distance - walls.left_distance;
            float setpoint = TARGET_CENTER_OFFSET;  // 通常は0 / Usually 0
            steering_angle = centeringPID.compute(setpoint, -error);
            break;
        }

        case MODE_LEFT_WALL: {
            // 目標: 左壁から一定距離 / Target: fixed distance from left wall
            float measured = walls.left_distance;
            steering_angle = wallFollowPID.compute(TARGET_LEFT_DISTANCE, measured);

            // 壁角度による補正も加える / Also add wall angle correction
            steering_angle += walls.left_angle * 0.5;
            break;
        }

        case MODE_RIGHT_WALL: {
            // 目標: 右壁から一定距離 / Target: fixed distance from right wall
            float measured = walls.right_distance;
            steering_angle = -wallFollowPID.compute(TARGET_LEFT_DISTANCE, measured);

            // 壁角度による補正 / Wall angle correction
            steering_angle -= walls.right_angle * 0.5;
            break;
        }

        case MODE_NO_WALLS:
        default:
            // 直進維持 / Maintain straight
            steering_angle = 0.0;
            break;
    }

    // =========================================================================
    // 安全制約: 左側センサーが近すぎる場合の緊急回避
    // Safety constraint: Emergency avoidance if left sensors too close
    // =========================================================================
    if (sensorData != nullptr) {
        uint16_t sensor0_dist = sensorData[0].valid ? sensorData[0].distance : 9999;
        uint16_t sensor1_dist = sensorData[1].valid ? sensorData[1].distance : 9999;
        uint16_t min_left_dist = min(sensor0_dist, sensor1_dist);

        if (min_left_dist < MIN_LEFT_DISTANCE) {
            float shortage = MIN_LEFT_DISTANCE - min_left_dist;
            steering_angle += shortage * LEFT_AVOID_GAIN;
        }
    }

    // 最大操舵角でクランプ / Clamp to max steering angle
    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    return steering_angle;
}

void SteeringController::reset() {
    centeringPID.reset();
    wallFollowPID.reset();
    currentMode = MODE_NO_WALLS;
    previousMode = MODE_NO_WALLS;
}

void SteeringController::printDebugInfo() const {
    Logger::print(" Mode:");
    switch (currentMode) {
        case MODE_BOTH_WALLS: Logger::print("BOTH"); break;
        case MODE_LEFT_WALL:  Logger::print("LEFT"); break;
        case MODE_RIGHT_WALL: Logger::print("RIGHT"); break;
        case MODE_NO_WALLS:   Logger::print("NONE"); break;
    }
}
