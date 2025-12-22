/*
 * test_integration.cpp
 *
 * WallDetector + SteeringController 結合テスト
 *
 * テスト対象:
 * - センサーデータ → 壁検出 → ステアリング計算 のパイプライン
 * - シナリオベースのテスト（コーナー進入、壁回復など）
 * - 時系列制御の安定性
 */

#include "test_common.h"

using namespace TestConstants;
using namespace TestHelpers;

class IntegrationTest : public BaseTestFixture {
protected:
    WallDetector detector;
    SteeringController controller;
    SensorData sensorData[5];

    void SetUp() override {
        BaseTestFixture::SetUp();
        controller.begin();

        for (int i = 0; i < 5; i++) {
            sensorData[i].distance = 0;
            sensorData[i].valid = true;
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
    }

    // フルパイプライン: センサー → 壁検出 → ステアリング
    float runPipeline() {
        WallDetection walls = detector.detect(sensorData);
        return controller.calculate(walls);
    }
};

// ============================================================================
// パイプラインテスト - 基本動作
// ============================================================================

TEST_F(IntegrationTest, Pipeline_BothWalls_CenterPosition) {
    // 左右対称なセンサー値 → 中央走行
    setSensorDistances(500, 350, 600, 350, 500);

    ArduinoMock::setMillis(0);
    runPipeline();

    ArduinoMock::setMillis(40);
    float steering = runPipeline();

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    EXPECT_NEAR(0.0f, steering, 5.0f);  // 中央なのでほぼ0
}

TEST_F(IntegrationTest, Pipeline_LeftBias_CorrectToRight) {
    // 左に寄っている（左センサーの方が近い）
    setSensorDistances(300, 250, 600, 450, 600);

    ArduinoMock::setMillis(0);
    runPipeline();

    ArduinoMock::setMillis(40);
    float steering = runPipeline();

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    EXPECT_GT(steering, 0.0f);  // 右へ補正
}

TEST_F(IntegrationTest, Pipeline_RightBias_CorrectToLeft) {
    // 右に寄っている
    setSensorDistances(600, 450, 600, 250, 300);

    ArduinoMock::setMillis(0);
    runPipeline();

    ArduinoMock::setMillis(40);
    float steering = runPipeline();

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    EXPECT_LT(steering, 0.0f);  // 左へ補正
}

TEST_F(IntegrationTest, Pipeline_LeftWallOnly) {
    // 右センサーが遠距離 → 左壁モード
    setSensorDistances(400, 350, 600, 1500, 1500);

    ArduinoMock::setMillis(0);
    runPipeline();

    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());
}

TEST_F(IntegrationTest, Pipeline_RightWallOnly) {
    // 左センサーが遠距離 → 右壁モード
    setSensorDistances(1500, 1500, 600, 350, 400);

    ArduinoMock::setMillis(0);
    runPipeline();

    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());
}

TEST_F(IntegrationTest, Pipeline_NoWalls) {
    // 全センサー遠距離
    setSensorDistances(1500, 1500, 600, 1500, 1500);

    ArduinoMock::setMillis(0);
    float steering = runPipeline();

    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());
    EXPECT_FLOAT_EQ(0.0f, steering);
}

TEST_F(IntegrationTest, Pipeline_EmergencyAvoidance_LeftWall) {
    // 左壁に非常に近い → 右へ大きくステア
    setSensorDistances(150, 100, 600, 600, 700);

    ArduinoMock::setMillis(0);
    runPipeline();

    ArduinoMock::setMillis(40);
    float steering = runPipeline();

    // 安全距離補正が大きく働く
    EXPECT_GT(steering, 15.0f);
}

TEST_F(IntegrationTest, Pipeline_EmergencyAvoidance_RightWall) {
    // 右壁に非常に近い
    setSensorDistances(700, 600, 600, 100, 150);

    ArduinoMock::setMillis(0);
    runPipeline();

    ArduinoMock::setMillis(40);
    float steering = runPipeline();

    EXPECT_LT(steering, -15.0f);
}

// ============================================================================
// シナリオテスト - 直線走路
// ============================================================================

TEST_F(IntegrationTest, Scenario_StraightCorridor_Stable) {
    // 直線走路（両壁300mm程度で平行）
    // d0(-70°) = 300/cos(70°) ≈ 877mm
    // d1(-20°) = 300/cos(20°) ≈ 319mm
    setSensorDistances(877, 319, 800, 319, 877);

    ArduinoMock::setMillis(0);
    runPipeline();

    std::vector<float> steerings;
    for (int i = 1; i <= 10; i++) {
        ArduinoMock::setMillis(i * 40);
        steerings.push_back(runPipeline());

        EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    }

    // 全ての出力が小さい（安定走行）
    for (float s : steerings) {
        EXPECT_LT(std::abs(s), 15.0f);
    }
}

TEST_F(IntegrationTest, Scenario_StraightCorridor_NoOscillation) {
    // 一定入力で振動しない
    setSensorDistances(500, 350, 600, 350, 500);

    ArduinoMock::setMillis(0);
    runPipeline();

    std::vector<float> outputs;
    for (int i = 1; i <= 30; i++) {
        ArduinoMock::setMillis(i * 40);
        outputs.push_back(runPipeline());
    }

    // 最後の5回の変化が小さい
    float max_change = 0.0f;
    for (size_t i = 25; i < 30; i++) {
        max_change = std::max(max_change, std::abs(outputs[i] - outputs[i-1]));
    }

    EXPECT_LT(max_change, 2.0f);
}

// ============================================================================
// シナリオテスト - コーナー進入
// ============================================================================

TEST_F(IntegrationTest, Scenario_LeftCorner_ModeTransition) {
    // 左コーナー進入: 両壁 → 右壁消失 → 左壁モード

    // Phase 1: 両壁検出
    setSensorDistances(500, 350, 600, 350, 500);
    ArduinoMock::setMillis(0);
    runPipeline();
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());

    // Phase 2: 右壁が遠くなり始める
    setSensorDistances(500, 350, 600, 600, 800);
    ArduinoMock::setMillis(40);
    runPipeline();
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());

    // Phase 3: 右壁消失
    setSensorDistances(500, 350, 600, 1500, 1500);
    ArduinoMock::setMillis(80);
    runPipeline();
    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());

    // Phase 4: 左壁追従継続
    ArduinoMock::setMillis(120);
    runPipeline();
    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());
}

TEST_F(IntegrationTest, Scenario_RightCorner_ModeTransition) {
    // 右コーナー進入

    setSensorDistances(500, 350, 600, 350, 500);
    ArduinoMock::setMillis(0);
    runPipeline();
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());

    setSensorDistances(1500, 1500, 600, 350, 500);
    ArduinoMock::setMillis(40);
    runPipeline();
    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());
}

// ============================================================================
// シナリオテスト - 壁回復
// ============================================================================

TEST_F(IntegrationTest, Scenario_WallRecovery_NoneToRight) {
    // 壁なし → 右壁出現

    setSensorDistances(1500, 1500, 600, 1500, 1500);
    ArduinoMock::setMillis(0);
    runPipeline();
    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());

    setSensorDistances(1500, 1500, 600, 350, 500);
    ArduinoMock::setMillis(40);
    runPipeline();
    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());
}

TEST_F(IntegrationTest, Scenario_WallRecovery_RightToBoth) {
    // 右壁 → 両壁出現

    setSensorDistances(1500, 1500, 600, 350, 500);
    ArduinoMock::setMillis(0);
    runPipeline();
    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());

    setSensorDistances(500, 350, 600, 350, 500);
    ArduinoMock::setMillis(40);
    runPipeline();
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

TEST_F(IntegrationTest, Scenario_WallRecovery_FullSequence) {
    // 壁なし → 右壁 → 両壁

    setSensorDistances(1500, 1500, 600, 1500, 1500);
    ArduinoMock::setMillis(0);
    runPipeline();
    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());

    setSensorDistances(1500, 1500, 600, 350, 500);
    ArduinoMock::setMillis(40);
    runPipeline();
    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());

    setSensorDistances(500, 350, 600, 350, 500);
    ArduinoMock::setMillis(80);
    runPipeline();
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

// ============================================================================
// 時系列制御テスト - 収束性
// ============================================================================

TEST_F(IntegrationTest, TimeSeries_Convergence_LeftBias) {
    // 左に偏った状態から徐々に収束

    std::vector<float> steering_history;
    float left_bias = 100.0f;  // 左に100mm偏り

    ArduinoMock::setMillis(0);
    // 偏った状態でスタート
    setSensorDistances(350, 280, 600, 380, 520);
    runPipeline();

    for (int i = 1; i <= 20; i++) {
        ArduinoMock::setMillis(i * 40);

        // 徐々に中央に近づく（ステアリング効果をシミュレート）
        left_bias *= 0.9f;
        int offset = (int)left_bias;

        setSensorDistances(
            450 - offset, 350 - offset/2, 600,
            350 + offset/2, 450 + offset
        );

        steering_history.push_back(runPipeline());
    }

    // 最後のステアリングは最初より小さい
    EXPECT_LT(std::abs(steering_history.back()),
              std::abs(steering_history.front()) + 5.0f);
}

TEST_F(IntegrationTest, TimeSeries_IntegralEffect) {
    // 積分項の効果確認（同じ偏差でも時間経過で出力変化）

    setSensorDistances(400, 320, 600, 380, 520);

    ArduinoMock::setMillis(0);
    runPipeline();

    float early_steering = 0.0f;
    float late_steering = 0.0f;

    for (int i = 1; i <= 50; i++) {
        ArduinoMock::setMillis(i * 40);
        float s = runPipeline();

        if (i == 5) early_steering = s;
        if (i == 50) late_steering = s;
    }

    // 積分項により後の出力が大きい
    EXPECT_GT(std::abs(late_steering), std::abs(early_steering) - 1.0f);
}

// ============================================================================
// 安定性・ロバスト性テスト
// ============================================================================

TEST_F(IntegrationTest, Robustness_SensorNoise_Small) {
    // 小さなセンサーノイズでも安定動作

    std::vector<float> steering_values;

    for (int i = 0; i <= 20; i++) {
        ArduinoMock::setMillis(i * 40);

        // ±10mmのノイズ
        int noise = (i % 2 == 0) ? 10 : -10;

        setSensorDistances(
            500 + noise, 350 + noise, 600,
            350 - noise, 500 - noise
        );

        float s = runPipeline();
        if (i > 0) {
            steering_values.push_back(s);
        }
    }

    // 全ての出力が有効範囲内
    for (float s : steering_values) {
        EXPECT_GE(s, -MAX_STEERING_ANGLE);
        EXPECT_LE(s, MAX_STEERING_ANGLE);
        EXPECT_FALSE(std::isnan(s));
    }
}

TEST_F(IntegrationTest, Robustness_SensorNoise_Large) {
    // 大きなノイズでもクラッシュしない

    for (int i = 0; i <= 30; i++) {
        ArduinoMock::setMillis(i * 40);

        // ±50mmのノイズ
        int noise = ((i % 3) - 1) * 50;

        setSensorDistances(
            500 + noise, 350 + noise/2, 600,
            350 - noise/2, 500 - noise
        );

        float s = runPipeline();

        EXPECT_FALSE(std::isnan(s));
        EXPECT_FALSE(std::isinf(s));
        EXPECT_GE(s, -MAX_STEERING_ANGLE);
        EXPECT_LE(s, MAX_STEERING_ANGLE);
    }
}

TEST_F(IntegrationTest, Robustness_ModeFlickering) {
    // モードが頻繁に切り替わっても安定

    for (int i = 0; i < 10; i++) {
        ArduinoMock::setMillis(i * 40);

        if (i % 2 == 0) {
            // 両壁モード
            setSensorDistances(500, 350, 600, 350, 500);
        } else {
            // 左壁モード
            setSensorDistances(500, 350, 600, 1500, 1500);
        }

        float s = runPipeline();

        EXPECT_GE(s, -MAX_STEERING_ANGLE);
        EXPECT_LE(s, MAX_STEERING_ANGLE);
    }
}

TEST_F(IntegrationTest, Robustness_RapidModeChanges) {
    // 急速なモード変更シーケンス

    ControlMode modes[] = {MODE_BOTH_WALLS, MODE_LEFT_WALL, MODE_RIGHT_WALL,
                           MODE_NO_WALLS, MODE_BOTH_WALLS};

    for (int i = 0; i < 5; i++) {
        ArduinoMock::setMillis(i * 40);

        switch (i) {
            case 0: setSensorDistances(500, 350, 600, 350, 500); break;  // Both
            case 1: setSensorDistances(500, 350, 600, 1500, 1500); break; // Left
            case 2: setSensorDistances(1500, 1500, 600, 350, 500); break; // Right
            case 3: setSensorDistances(1500, 1500, 600, 1500, 1500); break; // None
            case 4: setSensorDistances(500, 350, 600, 350, 500); break;  // Both
        }

        float s = runPipeline();
        EXPECT_EQ(modes[i], controller.getMode());
        EXPECT_FALSE(std::isnan(s));
    }
}

// ============================================================================
// 境界値テスト
// ============================================================================

TEST_F(IntegrationTest, Boundary_MinValidDistance) {
    // 最小有効距離でのパイプライン動作
    setSensorDistances(50, 55, 500, 55, 50);

    ArduinoMock::setMillis(0);
    float s = runPipeline();

    // 両壁が検出される（センサー差分は有効範囲内）
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    EXPECT_FALSE(std::isnan(s));
}

TEST_F(IntegrationTest, Boundary_MaxValidDistance) {
    // 最大有効距離でのパイプライン動作
    setSensorDistances(1200, 1150, 500, 1150, 1200);

    ArduinoMock::setMillis(0);
    float s = runPipeline();

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    EXPECT_FALSE(std::isnan(s));
}

TEST_F(IntegrationTest, Boundary_SensorDiffLimit) {
    // センサー差分が限界値付近
    // 差分599mm: 有効
    setSensorDistances(800, 201, 500, 201, 800);

    ArduinoMock::setMillis(0);
    runPipeline();

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

TEST_F(IntegrationTest, Boundary_SensorDiffExceeds) {
    // センサー差分が限界超過
    // 差分601mm: 無効
    setSensorDistances(801, 200, 500, 200, 801);

    ArduinoMock::setMillis(0);
    runPipeline();

    // 壁検出が無効になる
    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());
}

// ============================================================================
// エッジケーステスト
// ============================================================================

TEST_F(IntegrationTest, EdgeCase_AllSensorsMinimum) {
    // 全センサーが最小値
    setSensorDistances(50, 50, 50, 50, 50);

    ArduinoMock::setMillis(0);
    float s = runPipeline();

    EXPECT_FALSE(std::isnan(s));
    EXPECT_FALSE(std::isinf(s));
}

TEST_F(IntegrationTest, EdgeCase_AllSensorsMaximum) {
    // 全センサーが最大値
    setSensorDistances(1200, 1200, 1200, 1200, 1200);

    ArduinoMock::setMillis(0);
    float s = runPipeline();

    EXPECT_FALSE(std::isnan(s));
}

TEST_F(IntegrationTest, EdgeCase_AsymmetricSensors) {
    // 極端に非対称
    setSensorDistances(100, 1200, 500, 1200, 100);

    ArduinoMock::setMillis(0);
    float s = runPipeline();

    // センサー差分が大きすぎて無効になる可能性
    EXPECT_FALSE(std::isnan(s));
}

TEST_F(IntegrationTest, EdgeCase_FirstCallReturnZero) {
    // 初回呼び出しはPIDが0を返す
    setSensorDistances(300, 250, 600, 400, 550);

    ArduinoMock::setMillis(0);
    float s = runPipeline();

    // 初回は0（PID初期化の仕様）
    EXPECT_FLOAT_EQ(0.0f, s);
}

// ============================================================================
// 長時間連続動作テスト
// ============================================================================

TEST_F(IntegrationTest, LongRun_Stability) {
    // 100回の連続動作で安定

    setSensorDistances(500, 350, 600, 350, 500);

    ArduinoMock::setMillis(0);
    runPipeline();

    for (int i = 1; i <= 100; i++) {
        ArduinoMock::setMillis(i * 40);

        // 小さな変動を加える
        int var = (i % 10) - 5;
        setSensorDistances(
            500 + var, 350 + var, 600,
            350 - var, 500 - var
        );

        float s = runPipeline();

        EXPECT_GE(s, -MAX_STEERING_ANGLE);
        EXPECT_LE(s, MAX_STEERING_ANGLE);
        EXPECT_FALSE(std::isnan(s));
        EXPECT_FALSE(std::isinf(s));
        EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    }
}

TEST_F(IntegrationTest, LongRun_IntegralNotOverflow) {
    // 長時間動作で積分値がオーバーフローしない

    setSensorDistances(400, 300, 600, 400, 550);

    ArduinoMock::setMillis(0);
    runPipeline();

    for (int i = 1; i <= 1000; i++) {
        ArduinoMock::setMillis(i * 40);
        float s = runPipeline();

        // 出力が常に範囲内
        EXPECT_GE(s, -MAX_STEERING_ANGLE);
        EXPECT_LE(s, MAX_STEERING_ANGLE);
    }

    // 積分値がアンチワインドアップで制限されている
    float integral = controller.getCenteringPID().getIntegralValue();
    EXPECT_LE(std::abs(integral), STEERING_INTEGRAL_MAX + 1.0f);
}

