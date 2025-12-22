/*
 * SteeringController.h
 *
 * ステアリング制御クラス（宣言）
 * PID制御による壁追従制御
 */

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "WallDetector.h"
#include "PIDController.h"

// 制御モード / Control modes
enum ControlMode {
    MODE_BOTH_WALLS,    // 両壁検出 → 中央走行 / Both walls → center driving
    MODE_LEFT_WALL,     // 左壁のみ → 左壁追従 / Left wall only → follow left
    MODE_RIGHT_WALL,    // 右壁のみ → 右壁追従 / Right wall only → follow right
    MODE_NO_WALLS       // 壁なし → 直進 / No walls → straight
};

class SteeringController {
private:
    PIDController _centeringPID;    // 中央走行用PID（両壁モード：距離制御）
    PIDController _anglePID;        // 角度制御用PID（片壁モード：壁と平行維持）
    ControlMode _currentMode;
    ControlMode _previousMode;

public:
    SteeringController();

    // 初期化 / Initialize
    void begin();

    // ステアリング角度を計算 / Calculate steering angle
    float calculate(const WallDetection& walls);

    // 現在のモードを取得 / Get current mode
    ControlMode getMode() const { return _currentMode; }

    // PIDをリセット / Reset PID
    void reset();

    // デバッグ情報 / Debug info
    void printDebugInfo() const;
};

#endif // STEERING_CONTROLLER_H
