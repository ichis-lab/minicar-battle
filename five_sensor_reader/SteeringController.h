/*
 * SteeringController.h
 *
 * シンプル状態ベース制御
 * - 直進モード: 右壁追従
 * - コーナリングモード: 開いている方向へ曲がる
 * - 緊急回避モード: 最大ステアリング
 */

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "SensorReader.h"

// 制御モード
enum ControlMode {
    MODE_STRAIGHT,      // 直進（右壁追従）
    MODE_CORNER,        // コーナリング
    MODE_EMERGENCY,     // 緊急回避
    MODE_SIDE_AVOID     // 側壁回避
};

class SteeringController {
private:
    ControlMode _currentMode;
    float _lastSteering;

public:
    SteeringController();

    // 初期化
    void begin();

    // ステアリング角度を計算（センサーデータから直接）
    float calculate(const SensorData* sensors);

    // 現在のモードを取得（デバッグ用）
    ControlMode getCurrentMode() const { return _currentMode; }

    // モード名を文字列で取得
    const char* getModeName() const;
};

#endif // STEERING_CONTROLLER_H
