/*
 * test_common.h
 *
 * テスト共通定義：定数、ヘルパー関数、テストフィクスチャ
 */

#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "Arduino.h"
#include "Config.h"
#include "SensorReader.h"
#include "WallDetector.h"
#include "PIDController.h"
#include "SteeringController.h"

// ============================================================================
// テスト用定数（マジックナンバー排除）
// ============================================================================
namespace TestConstants {
    // 制御周期
    constexpr unsigned long CONTROL_PERIOD_MS = 40;  // 25Hz

    // センサー角度（度）
    constexpr float SENSOR_0_ANGLE = -70.0f;
    constexpr float SENSOR_1_ANGLE = -20.0f;
    constexpr float SENSOR_3_ANGLE = 20.0f;
    constexpr float SENSOR_4_ANGLE = 70.0f;

    // 三角関数値（事前計算）
    constexpr float SIN_70 = 0.9397f;
    constexpr float COS_70 = 0.3420f;
    constexpr float SIN_20 = 0.3420f;
    constexpr float COS_20 = 0.9397f;

    // 壁距離テスト用
    constexpr float WALL_DISTANCE_300MM = 300.0f;
    constexpr float WALL_DISTANCE_TOLERANCE = 10.0f;
    constexpr float WALL_ANGLE_TOLERANCE = 3.0f;

    // 境界値
    constexpr uint16_t SENSOR_MIN_VALID = MIN_VALID_DISTANCE;      // 50mm
    constexpr uint16_t SENSOR_MAX_VALID = RELIABLE_RANGE;          // 1200mm
    constexpr uint16_t SENSOR_DIFF_LIMIT = MAX_SENSOR_DIFF;        // 600mm
}

// ============================================================================
// センサーデータ生成ヘルパー
// ============================================================================
namespace TestHelpers {

    // センサーデータ配列を初期化
    inline void initSensorData(SensorData* data, uint16_t distances[5]) {
        for (int i = 0; i < 5; i++) {
            data[i].distance = distances[i];
            data[i].valid = true;
            data[i].status = 0;
        }
    }

    // 平行な壁のセンサー距離を計算（手計算による期待値生成）
    // 壁距離: wall_dist (mm), 壁角度: 0°（平行）
    inline void calcParallelWallDistances(float wall_dist,
                                          uint16_t& dist_far,
                                          uint16_t& dist_near,
                                          bool is_left_wall) {
        // 平行な壁の場合、各センサーの距離は:
        // distance = wall_dist / cos(sensor_angle)
        // ただし壁が左の場合はセンサー角度は負、右の場合は正
        float angle_far = is_left_wall ? TestConstants::SENSOR_0_ANGLE : TestConstants::SENSOR_4_ANGLE;
        float angle_near = is_left_wall ? TestConstants::SENSOR_1_ANGLE : TestConstants::SENSOR_3_ANGLE;

        // cos(angle)の絶対値を使う
        dist_far = static_cast<uint16_t>(wall_dist / TestConstants::COS_70);
        dist_near = static_cast<uint16_t>(wall_dist / TestConstants::COS_20);
    }

    // 壁の垂直距離を手計算（2点から原点への距離）
    // P1(x1,y1), P2(x2,y2)を通る直線と原点(0,0)の距離
    inline float calcWallDistance(float x1, float y1, float x2, float y2) {
        float dx = x2 - x1;
        float dy = y2 - y1;
        float line_length = std::sqrt(dx * dx + dy * dy);
        if (line_length < 0.001f) return 0.0f;

        float cross_product = x1 * y2 - x2 * y1;
        return std::abs(cross_product) / line_length;
    }

    // 壁の角度を手計算
    inline float calcWallAngle(float x1, float y1, float x2, float y2) {
        float dx = x2 - x1;
        float dy = y2 - y1;
        if (std::abs(dy) < 0.001f) return 0.0f;
        return std::atan2(dx, dy) * 180.0f / M_PI;
    }

    // センサー距離から検出点の座標を計算
    inline void calcSensorPoint(uint16_t distance, float angle_deg,
                                float& x, float& y) {
        float rad = angle_deg * M_PI / 180.0f;
        x = distance * std::sin(rad);
        y = distance * std::cos(rad);
    }

    // WallDetection構造体を作成
    inline WallDetection createWallDetection(
        bool left_valid, bool right_valid,
        float left_distance, float right_distance,
        float left_angle, float right_angle) {
        WallDetection walls;
        walls.left_valid = left_valid;
        walls.right_valid = right_valid;
        walls.left_distance = left_distance;
        walls.right_distance = right_distance;
        walls.left_angle = left_angle;
        walls.right_angle = right_angle;
        return walls;
    }

    // PID計算を複数回実行（時間を進めながら）
    inline std::vector<float> runPIDSequence(
        PIDController& pid,
        float setpoint,
        const std::vector<float>& measurements,
        unsigned long start_time = 0) {

        std::vector<float> outputs;
        ArduinoMock::setMillis(start_time);
        pid.compute(setpoint, measurements[0]);  // 初回（0を返す）

        for (size_t i = 1; i < measurements.size(); i++) {
            ArduinoMock::advanceMillis(TestConstants::CONTROL_PERIOD_MS);
            outputs.push_back(pid.compute(setpoint, measurements[i]));
        }
        return outputs;
    }
}

// ============================================================================
// 基底テストフィクスチャ
// ============================================================================
class BaseTestFixture : public ::testing::Test {
protected:
    void SetUp() override {
        ArduinoMock::resetMillis();
    }

    void TearDown() override {
        ArduinoMock::resetMillis();
    }

    // 時間を進めてPID計算（初回スキップ済み想定）
    void advanceAndCompute(PIDController& pid, float setpoint, float measured,
                           unsigned long advance_ms = TestConstants::CONTROL_PERIOD_MS) {
        ArduinoMock::advanceMillis(advance_ms);
        pid.compute(setpoint, measured);
    }
};

#endif // TEST_COMMON_H
