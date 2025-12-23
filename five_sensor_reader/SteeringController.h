/*
 * SteeringController.h
 *
 * ステアリング制御クラス（宣言）
 * 角度ベース統一PID制御による壁追従
 */

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "WallDetector.h"
#include "PIDController.h"

// 制御モード（デバッグ表示用）
enum ControlMode {
    MODE_BOTH_WALLS,    // 両壁検出 → 中央走行
    MODE_LEFT_WALL,     // 左壁のみ → 左壁追従
    MODE_RIGHT_WALL,    // 右壁のみ → 右壁追従
    MODE_NO_WALLS       // 壁なし → 直進
};

class SteeringController {
private:
    PIDController _pid;             // 統一PID（角度ベース）
    float _lastError;               // 最後のPID入力エラー値

public:
    SteeringController();

    // 初期化
    void begin();

    // ステアリング角度を計算
    float calculate(const WallDetection& walls);

    // 最後のエラー値を取得（デバッグ用）
    float getLastError() const { return _lastError; }

    // PIDをリセット
    void reset();
};

#endif // STEERING_CONTROLLER_H
