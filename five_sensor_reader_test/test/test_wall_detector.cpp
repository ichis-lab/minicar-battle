/*
 * test_wall_detector.cpp
 *
 * WallDetector単体テスト
 *
 * 仕様:
 * - センサー配置: Index 0(-70°), 1(-20°), 2(0°), 3(+20°), 4(+70°)
 * - 左壁検出: センサー0,1のペア
 * - 右壁検出: センサー4,3のペア
 * - 壁角度: angle > 0 = 前方で壁に近づく, angle < 0 = 前方で壁から離れる
 */

#include "test_common.h"

using namespace TestConstants;
using namespace TestHelpers;

class WallDetectorTest : public BaseTestFixture {
protected:
    WallDetector detector;
    SensorData sensorData[5];

    void SetUp() override {
        BaseTestFixture::SetUp();
        for (int i = 0; i < 5; i++) {
            sensorData[i].distance = 0;
            sensorData[i].valid = false;
            sensorData[i].status = 0;
        }
    }

    void setSensorDistances(uint16_t d0, uint16_t d1, uint16_t d2,
                            uint16_t d3, uint16_t d4) {
        sensorData[0].distance = d0;
        sensorData[1].distance = d1;
        sensorData[2].distance = d2;
        sensorData[3].distance = d3;
        sensorData[4].distance = d4;
        for (int i = 0; i < 5; i++) {
            sensorData[i].valid = true;
        }
    }

    // 手計算で壁の距離と角度を求める
    void calcExpectedWall(uint16_t dist_far, uint16_t dist_near,
                          float angle_far, float angle_near,
                          float& expected_distance, float& expected_angle) {
        float x1, y1, x2, y2;
        calcSensorPoint(dist_far, angle_far, x1, y1);
        calcSensorPoint(dist_near, angle_near, x2, y2);
        expected_distance = calcWallDistance(x1, y1, x2, y2);
        expected_angle = calcWallAngle(x1, y1, x2, y2);
    }
};

// ============================================================================
// 境界値テスト - センサー有効範囲
// ============================================================================

TEST_F(WallDetectorTest, BoundaryValue_MinDistance_Valid) {
    // MIN_VALID_DISTANCE(50mm)は有効
    setSensorDistances(50, 55, 500, 55, 50);
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
}

TEST_F(WallDetectorTest, BoundaryValue_MinDistanceMinus1_Invalid) {
    // 49mmは無効
    setSensorDistances(49, 55, 500, 55, 49);
    WallDetection result = detector.detect(sensorData);
    EXPECT_FALSE(result.left_valid);
    EXPECT_FALSE(result.right_valid);
}

TEST_F(WallDetectorTest, BoundaryValue_MaxDistance_Valid) {
    // RELIABLE_RANGE(1200mm)は有効
    setSensorDistances(1200, 1150, 500, 1150, 1200);
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
}

TEST_F(WallDetectorTest, BoundaryValue_MaxDistancePlus1_Invalid) {
    // 1201mmは無効
    setSensorDistances(1201, 1150, 500, 1150, 1201);
    WallDetection result = detector.detect(sensorData);
    EXPECT_FALSE(result.left_valid);
    EXPECT_FALSE(result.right_valid);
}

// ============================================================================
// 境界値テスト - センサーペア差分
// ============================================================================

TEST_F(WallDetectorTest, BoundaryValue_DiffAtLimit_Valid) {
    // 差分599mmは有効（> MAX_SENSOR_DIFF で比較）
    setSensorDistances(800, 201, 500, 201, 800);  // 差分599mm
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
}

TEST_F(WallDetectorTest, BoundaryValue_DiffExceedsLimit_Invalid) {
    // 差分601mmは無効
    setSensorDistances(801, 200, 500, 200, 801);  // 差分601mm
    WallDetection result = detector.detect(sensorData);
    EXPECT_FALSE(result.left_valid);
    EXPECT_FALSE(result.right_valid);
}

// ============================================================================
// 幾何学計算テスト - 手計算による期待値検証
// ============================================================================

TEST_F(WallDetectorTest, Geometry_ParallelLeftWall_300mm) {
    // 左壁が300mmの距離で平行な場合
    // センサー0(-70°): dist = 300/cos(70°) ≈ 877mm
    // センサー1(-20°): dist = 300/cos(20°) ≈ 319mm
    uint16_t dist0 = static_cast<uint16_t>(300.0f / COS_70);  // ≈877
    uint16_t dist1 = static_cast<uint16_t>(300.0f / COS_20);  // ≈319
    setSensorDistances(dist0, dist1, 500, 500, 500);

    // 手計算で期待値を求める
    float expected_dist, expected_angle;
    calcExpectedWall(dist0, dist1, SENSOR_0_ANGLE, SENSOR_1_ANGLE,
                     expected_dist, expected_angle);

    WallDetection result = detector.detect(sensorData);

    EXPECT_TRUE(result.left_valid);
    EXPECT_NEAR(expected_dist, result.left_distance, WALL_DISTANCE_TOLERANCE);
    EXPECT_NEAR(expected_angle, result.left_angle, WALL_ANGLE_TOLERANCE);
}

TEST_F(WallDetectorTest, Geometry_ParallelRightWall_300mm) {
    // 右壁が300mmの距離で平行な場合
    uint16_t dist4 = static_cast<uint16_t>(300.0f / COS_70);
    uint16_t dist3 = static_cast<uint16_t>(300.0f / COS_20);
    setSensorDistances(500, 500, 500, dist3, dist4);

    float expected_dist, expected_angle;
    calcExpectedWall(dist4, dist3, SENSOR_4_ANGLE, SENSOR_3_ANGLE,
                     expected_dist, expected_angle);

    WallDetection result = detector.detect(sensorData);

    EXPECT_TRUE(result.right_valid);
    EXPECT_NEAR(expected_dist, result.right_distance, WALL_DISTANCE_TOLERANCE);
    EXPECT_NEAR(expected_angle, result.right_angle, WALL_ANGLE_TOLERANCE);
}

TEST_F(WallDetectorTest, Geometry_ApproachingLeftWall) {
    // 左壁に向かっている: センサー1(前向き)がセンサー0(横向き)より近い
    // → 壁が右上がり → angle > 0
    uint16_t dist0 = 500;  // 横向きセンサー: 遠い
    uint16_t dist1 = 280;  // 前向きセンサー: 近い
    setSensorDistances(dist0, dist1, 500, 500, 500);

    float expected_dist, expected_angle;
    calcExpectedWall(dist0, dist1, SENSOR_0_ANGLE, SENSOR_1_ANGLE,
                     expected_dist, expected_angle);

    WallDetection result = detector.detect(sensorData);

    EXPECT_TRUE(result.left_valid);
    EXPECT_GT(result.left_angle, 0.0f);  // 壁に向かっている
    EXPECT_NEAR(expected_angle, result.left_angle, WALL_ANGLE_TOLERANCE);
}

TEST_F(WallDetectorTest, Geometry_DepartingLeftWall) {
    // 左壁から離れている: angle < 0
    // dx < 0 になる条件: dist_near > 2.75 * dist_far
    uint16_t dist0 = 200;  // 横向きセンサー
    uint16_t dist1 = 600;  // 前向きセンサー: dist1 > 2.75 * dist0
    setSensorDistances(dist0, dist1, 500, 500, 500);

    float expected_dist, expected_angle;
    calcExpectedWall(dist0, dist1, SENSOR_0_ANGLE, SENSOR_1_ANGLE,
                     expected_dist, expected_angle);

    WallDetection result = detector.detect(sensorData);

    EXPECT_TRUE(result.left_valid);
    EXPECT_LT(result.left_angle, 0.0f);  // 壁から離れている
    EXPECT_NEAR(expected_angle, result.left_angle, WALL_ANGLE_TOLERANCE);
}

TEST_F(WallDetectorTest, Geometry_ApproachingRightWall) {
    // 右壁に向かっている: センサー3(前向き)がセンサー4(横向き)より近い
    uint16_t dist4 = 500;
    uint16_t dist3 = 280;
    setSensorDistances(500, 500, 500, dist3, dist4);

    float expected_dist, expected_angle;
    calcExpectedWall(dist4, dist3, SENSOR_4_ANGLE, SENSOR_3_ANGLE,
                     expected_dist, expected_angle);

    WallDetection result = detector.detect(sensorData);

    EXPECT_TRUE(result.right_valid);
    EXPECT_LT(result.right_angle, 0.0f);  // 右壁の場合、符号が逆
    EXPECT_NEAR(expected_angle, result.right_angle, WALL_ANGLE_TOLERANCE);
}

TEST_F(WallDetectorTest, Geometry_DepartingRightWall) {
    // 右壁から離れている
    uint16_t dist4 = 200;
    uint16_t dist3 = 600;
    setSensorDistances(500, 500, 500, dist3, dist4);

    float expected_dist, expected_angle;
    calcExpectedWall(dist4, dist3, SENSOR_4_ANGLE, SENSOR_3_ANGLE,
                     expected_dist, expected_angle);

    WallDetection result = detector.detect(sensorData);

    EXPECT_TRUE(result.right_valid);
    EXPECT_GT(result.right_angle, 0.0f);  // 右壁の場合、符号が逆
    EXPECT_NEAR(expected_angle, result.right_angle, WALL_ANGLE_TOLERANCE);
}

// ============================================================================
// 壁検出結果テスト
// ============================================================================

TEST_F(WallDetectorTest, Detection_BothWalls) {
    setSensorDistances(400, 350, 500, 350, 400);
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
}

TEST_F(WallDetectorTest, Detection_LeftWallOnly) {
    setSensorDistances(400, 350, 500, 1500, 1500);
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_FALSE(result.right_valid);
}

TEST_F(WallDetectorTest, Detection_RightWallOnly) {
    setSensorDistances(1500, 1500, 500, 350, 400);
    WallDetection result = detector.detect(sensorData);
    EXPECT_FALSE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
}

TEST_F(WallDetectorTest, Detection_NoWalls) {
    setSensorDistances(1500, 1500, 500, 1500, 1500);
    WallDetection result = detector.detect(sensorData);
    EXPECT_FALSE(result.left_valid);
    EXPECT_FALSE(result.right_valid);
}

// ============================================================================
// 異常系テスト
// ============================================================================

TEST_F(WallDetectorTest, Abnormal_NullPointer) {
    WallDetection result = detector.detect(nullptr);
    EXPECT_FALSE(result.left_valid);
    EXPECT_FALSE(result.right_valid);
    EXPECT_FLOAT_EQ(0.0f, result.left_distance);
    EXPECT_FLOAT_EQ(0.0f, result.right_distance);
}

TEST_F(WallDetectorTest, Abnormal_AllSensorsAtMinimum) {
    // 全センサーが最小値
    setSensorDistances(50, 50, 50, 50, 50);
    WallDetection result = detector.detect(sensorData);
    // 差分が0なので有効だが、計算結果は有効であること
    EXPECT_FALSE(std::isnan(result.left_distance));
    EXPECT_FALSE(std::isnan(result.right_distance));
}

TEST_F(WallDetectorTest, Abnormal_SameSensorValues) {
    // 両センサーが同じ距離（特殊なケース）
    setSensorDistances(400, 400, 500, 400, 400);
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
    // 距離と角度が有効な値
    EXPECT_GT(result.left_distance, 0.0f);
    EXPECT_GT(result.right_distance, 0.0f);
}

TEST_F(WallDetectorTest, Abnormal_OneSensorInvalid) {
    // 片方のセンサーだけ無効
    setSensorDistances(30, 400, 500, 400, 30);  // センサー0,4が無効
    WallDetection result = detector.detect(sensorData);
    EXPECT_FALSE(result.left_valid);
    EXPECT_FALSE(result.right_valid);
}

// ============================================================================
// 壁距離の一貫性テスト
// ============================================================================

TEST_F(WallDetectorTest, Consistency_SymmetricWalls) {
    // 左右対称な配置
    uint16_t dist_far = 500;
    uint16_t dist_near = 350;
    setSensorDistances(dist_far, dist_near, 600, dist_near, dist_far);

    WallDetection result = detector.detect(sensorData);

    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
    // 対称なので距離はほぼ同じ
    EXPECT_NEAR(result.left_distance, result.right_distance, 5.0f);
    // 角度は符号が逆（左右対称）
    EXPECT_NEAR(std::abs(result.left_angle), std::abs(result.right_angle), 2.0f);
}

TEST_F(WallDetectorTest, Consistency_CloserLeftWall) {
    // 左壁が近い配置
    setSensorDistances(300, 200, 500, 600, 800);
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
    EXPECT_LT(result.left_distance, result.right_distance);
}

TEST_F(WallDetectorTest, Consistency_CloserRightWall) {
    // 右壁が近い配置
    setSensorDistances(800, 600, 500, 200, 300);
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
    EXPECT_GT(result.left_distance, result.right_distance);
}

// ============================================================================
// エッジケーステスト
// ============================================================================

TEST_F(WallDetectorTest, EdgeCase_VeryCloseWall) {
    // 非常に近い壁（50mm付近）
    uint16_t dist0 = static_cast<uint16_t>(50.0f / COS_70);  // ≈146mm
    uint16_t dist1 = static_cast<uint16_t>(50.0f / COS_20);  // ≈53mm
    setSensorDistances(dist0, dist1, 100, dist1, dist0);

    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);
    EXPECT_TRUE(result.right_valid);
    EXPECT_NEAR(50.0f, result.left_distance, 15.0f);
}

TEST_F(WallDetectorTest, EdgeCase_VeryFarWall) {
    // 遠い壁（1000mm付近）
    uint16_t dist0 = static_cast<uint16_t>(1000.0f / COS_70);  // 範囲外になる可能性
    uint16_t dist1 = static_cast<uint16_t>(1000.0f / COS_20);  // ≈1064mm

    // dist0が範囲外(>1200)になる場合は無効
    if (dist0 > RELIABLE_RANGE) {
        setSensorDistances(dist0, dist1, 500, dist1, dist0);
        WallDetection result = detector.detect(sensorData);
        EXPECT_FALSE(result.left_valid);
    }
}

TEST_F(WallDetectorTest, EdgeCase_LargeSensorDiff) {
    // 差分がちょうど限界値付近
    setSensorDistances(850, 251, 500, 251, 850);  // 差分599mm
    WallDetection result = detector.detect(sensorData);
    EXPECT_TRUE(result.left_valid);  // ギリギリ有効
}
