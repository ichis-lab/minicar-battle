/*
 * SteeringController.h
 *
 * ステアリング制御クラス（宣言）
 * 開放度ベースPID制御
 *
 * Steering Controller (Openness-based PID)
 */

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "OpennessCalculator.h"
#include "PIDController.h"

// 前方宣言 / Forward declaration
struct SensorData;

/**
 * ステアリング制御クラス（開放度ベース）
 * Steering Controller (Openness-based)
 */
class SteeringController {
private:
    OpennessCalculator opennessCalc;
    PIDController pid;

    // 最後に計算した開放度データ（デバッグ用）
    OpennessData lastOpennessData;

    /**
     * 緊急回避が必要か判定
     * Check if emergency avoidance is needed
     */
    bool needsEmergencyAvoidance(uint16_t front_distance);

    /**
     * 緊急回避時のステアリング計算
     * Calculate steering for emergency avoidance
     */
    float calculateEmergencySteering(const OpennessData& openness);

public:
    /**
     * コンストラクタ
     */
    SteeringController();

    /**
     * ステアリング角度を計算
     * Calculate steering angle
     * @param sensorData センサーデータ配列[5]
     * @return ステアリング角度（度） / Steering angle in degrees
     */
    float calculate(const SensorData* sensorData);

    /**
     * 状態リセット
     * Reset state
     */
    void reset();

    /**
     * 最後の開放度データを取得（デバッグ用）
     * Get last openness data (for debugging)
     */
    const OpennessData& getLastOpennessData() const;
};

#endif // STEERING_CONTROLLER_H
