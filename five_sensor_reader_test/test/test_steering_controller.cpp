/*
 * test_steering_controller.cpp
 *
 * SteeringController単体テスト
 *
 * 仕様:
 * - 両壁モード: error = right - left, steering = centeringPID(0, -(right-left))
 *   → 左寄り(left < right) → 右へステア(正)
 * - 左壁モード: steering = -anglePID(0, left_angle)
 *   → 壁角度 > 0（壁に向かう）→ 右へステア（正）
 *   → 壁角度 < 0（壁から離れる）→ 左へステア（負）
 *   → 安全距離未満で右方向に補正を加算
 * - 右壁モード: steering = -anglePID(0, right_angle)
 *   → 壁角度 > 0（壁に向かう）→ 左へステア（負）
 *   → 壁角度 < 0（壁から離れる）→ 右へステア（正）
 *   → 安全距離未満で左方向に補正を減算
 * - 壁なし: steering = 0
 *
 * PIDゲイン（Config.h）:
 * - 中央PID: Kp=0.08, Ki=0.005, Kd=0.02
 * - 角度PID: Kp=0.8, Ki=0, Kd=0
 * - 安全距離: MIN_SAFE_DISTANCE=300mm, DISTANCE_AVOID_GAIN=0.15
 */

#include "test_common.h"

using namespace TestConstants;
using namespace TestHelpers;

class SteeringControllerTest : public BaseTestFixture {
protected:
    SteeringController controller;

    void SetUp() override {
        BaseTestFixture::SetUp();
        controller.begin();
    }
};

// ============================================================================
// モード判定テスト
// ============================================================================

TEST_F(SteeringControllerTest, Mode_BothWalls_Valid) {
    WallDetection walls = createWallDetection(true, true, 300, 300, 0, 0);
    controller.calculate(walls);
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

TEST_F(SteeringControllerTest, Mode_LeftWallOnly_Valid) {
    WallDetection walls = createWallDetection(true, false, 300, 0, 0, 0);
    controller.calculate(walls);
    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());
}

TEST_F(SteeringControllerTest, Mode_RightWallOnly_Valid) {
    WallDetection walls = createWallDetection(false, true, 0, 300, 0, 0);
    controller.calculate(walls);
    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());
}

TEST_F(SteeringControllerTest, Mode_NoWalls_Valid) {
    WallDetection walls = createWallDetection(false, false, 0, 0, 0, 0);
    controller.calculate(walls);
    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());
}

TEST_F(SteeringControllerTest, Mode_Priority_BothOverLeft) {
    // 両方有効なら両壁モードを選択
    WallDetection walls = createWallDetection(true, true, 300, 400, 5, -5);
    controller.calculate(walls);
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

// ============================================================================
// 両壁モード（距離PID）テスト - 手計算による期待値検証
// ============================================================================

TEST_F(SteeringControllerTest, BothWalls_Center_ZeroSteering) {
    // 左右等距離: 中央にいる
    // error = right - left = 300 - 300 = 0
    // steering = centeringPID(0, -(0)) = centeringPID(0, 0) = 0
    WallDetection walls = createWallDetection(true, true, 300, 300, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_NEAR(0.0f, steering, 1.0f);
}

TEST_F(SteeringControllerTest, BothWalls_LeftBias_SteerRight) {
    // 左寄り: left_dist < right_dist → 右へ（正）
    // error = right - left = 400 - 200 = 200
    // PID measured = -(200) = -200
    // PID error = 0 - (-200) = 200
    // P = 0.08 * 200 = 16
    WallDetection walls = createWallDetection(true, true, 200, 400, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // P = 0.08 * 200 = 16 (不感帯10mm外なので有効)
    // I項は小さいので無視
    EXPECT_GT(steering, 10.0f);
    EXPECT_LT(steering, 25.0f);
}

TEST_F(SteeringControllerTest, BothWalls_RightBias_SteerLeft) {
    // 右寄り: left_dist > right_dist → 左へ（負）
    // error = right - left = 200 - 400 = -200
    // PID measured = -(-200) = 200
    // PID error = 0 - 200 = -200
    // P = 0.08 * (-200) = -16
    WallDetection walls = createWallDetection(true, true, 400, 200, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_LT(steering, -10.0f);
    EXPECT_GT(steering, -25.0f);
}

TEST_F(SteeringControllerTest, BothWalls_InsideDeadband_SmallSteering) {
    // 不感帯内: 差分 < 10mm
    // error = 304 - 296 = 8 → 不感帯内
    // 注: 初回のprev_errorが設定されるため、2回目のD項が発生する
    // D = Kd * filtered_derivative で、初回error=8から2回目error=0への変化でD項が非ゼロ
    // そのため、完全なゼロにはならないが、P項とI項は0
    WallDetection walls = createWallDetection(true, true, 296, 304, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    // 数回呼び出してD項を収束させる
    for (int i = 1; i <= 5; i++) {
        ArduinoMock::setMillis(i * 40);
        controller.calculate(walls);
    }

    ArduinoMock::setMillis(240);
    float steering = controller.calculate(walls);

    // D項が収束した後はほぼ0
    EXPECT_NEAR(0.0f, steering, 2.0f);
}

TEST_F(SteeringControllerTest, BothWalls_LargeBias_ClampedToMax) {
    // 極端な偏り: MAX_STEERING_ANGLEでクランプ
    // error = 800 - 100 = 700
    // P = 0.08 * 700 = 56 → 30にクランプ
    WallDetection walls = createWallDetection(true, true, 100, 800, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_FLOAT_EQ(MAX_STEERING_ANGLE, steering);
}

TEST_F(SteeringControllerTest, BothWalls_LargeBiasNegative_ClampedToMin) {
    WallDetection walls = createWallDetection(true, true, 800, 100, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_FLOAT_EQ(-MAX_STEERING_ANGLE, steering);
}

TEST_F(SteeringControllerTest, BothWalls_IntegralAccumulation) {
    // 一定偏差で積分項が蓄積
    WallDetection walls = createWallDetection(true, true, 250, 350, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    float first_steering = 0.0f;
    float last_steering = 0.0f;

    for (int i = 1; i <= 20; i++) {
        ArduinoMock::setMillis(i * 40);
        float steering = controller.calculate(walls);
        if (i == 1) first_steering = steering;
        if (i == 20) last_steering = steering;
    }

    // 積分項により徐々に出力が増加（同じ偏差でも時間経過で変化）
    EXPECT_GT(last_steering, first_steering);
}

// ============================================================================
// 左壁モード（角度PID）テスト
// ============================================================================

TEST_F(SteeringControllerTest, LeftWall_Parallel_ZeroSteering) {
    // 壁と平行: angle = 0°
    // steering = anglePID(0, 0) = 0
    WallDetection walls = createWallDetection(true, false, 400, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_NEAR(0.0f, steering, 2.0f);
}

TEST_F(SteeringControllerTest, LeftWall_PositiveAngle_PIDResponse) {
    // angle > 0（壁に向かっている）
    // PID: error = 0 - 10 = -10, P = 0.8 * (-10) = -8
    // 実装: steering = -anglePID(0, 10) = -(-8) = 8（右へ）
    WallDetection walls = createWallDetection(true, false, 400, 0, 10, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // 壁に向かっている → 右へステア（正）
    EXPECT_GT(steering, 5.0f);
}

TEST_F(SteeringControllerTest, LeftWall_NegativeAngle_PIDResponse) {
    // angle < 0（壁から離れている）
    // PID: error = 0 - (-10) = 10, P = 0.8 * 10 = 8
    // 実装: steering = -anglePID(0, -10) = -(8) = -8（左へ）
    WallDetection walls = createWallDetection(true, false, 400, 0, -10, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // 壁から離れている → 左へステア（負）
    EXPECT_LT(steering, -5.0f);
}

TEST_F(SteeringControllerTest, LeftWall_SafetyDistance_Correction) {
    // 安全距離未満: 200mm < 300mm
    // shortage = 300 - 200 = 100mm
    // 補正 = 100 * 0.15 = 15°（右へ）
    // 角度0°なのでPID出力は0 → steering = 0 + 15 = 15
    WallDetection walls = createWallDetection(true, false, 200, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // 安全距離補正: (300-200)*0.15 = 15
    EXPECT_NEAR(15.0f, steering, 3.0f);
}

TEST_F(SteeringControllerTest, LeftWall_VeryCLose_LargeCorrection) {
    // 非常に近い: 100mm
    // shortage = 300 - 100 = 200mm
    // 補正 = 200 * 0.15 = 30°
    WallDetection walls = createWallDetection(true, false, 100, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_NEAR(MAX_STEERING_ANGLE, steering, 5.0f);
}

TEST_F(SteeringControllerTest, LeftWall_SafeDistance_NoCorrection) {
    // 安全距離以上: 350mm >= 300mm → 補正なし
    WallDetection walls = createWallDetection(true, false, 350, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // 角度0、距離安全 → ステアリングほぼ0
    EXPECT_NEAR(0.0f, steering, 2.0f);
}

TEST_F(SteeringControllerTest, LeftWall_AnglePlusSafety_Combined) {
    // 角度あり + 安全距離未満
    // angle = 5°, distance = 200mm
    // PID: error = -5, P = -4
    // Safety: +15
    // Total = -4 + 15 = 11
    WallDetection walls = createWallDetection(true, false, 200, 0, 5, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // PID出力 + 安全距離補正
    EXPECT_GT(steering, 5.0f);
}

// ============================================================================
// 右壁モード（角度PID）テスト
// ============================================================================

TEST_F(SteeringControllerTest, RightWall_Parallel_ZeroSteering) {
    WallDetection walls = createWallDetection(false, true, 0, 400, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_NEAR(0.0f, steering, 2.0f);
}

TEST_F(SteeringControllerTest, RightWall_PositiveAngle_PIDResponse) {
    // angle > 0（壁に向かっている）
    // PID: error = 0 - 10 = -10, output = -8
    // steering = -(-8) = 8
    // 実装: steering = -anglePID(0, 10) → 正の出力
    WallDetection walls = createWallDetection(false, true, 0, 400, 0, 10);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // 右壁のangle > 0 → 正のステアリング（左へ...の意図だが符号反転で正）
    EXPECT_GT(steering, 5.0f);
}

TEST_F(SteeringControllerTest, RightWall_NegativeAngle_PIDResponse) {
    // angle < 0（壁から離れている）
    WallDetection walls = createWallDetection(false, true, 0, 400, 0, -10);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_LT(steering, -5.0f);
}

TEST_F(SteeringControllerTest, RightWall_SafetyDistance_Correction) {
    // 安全距離未満: 200mm < 300mm
    // shortage = 100mm, 補正 = -15°（左へ）
    WallDetection walls = createWallDetection(false, true, 0, 200, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_NEAR(-15.0f, steering, 3.0f);
}

TEST_F(SteeringControllerTest, RightWall_VeryClose_LargeCorrection) {
    WallDetection walls = createWallDetection(false, true, 0, 100, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_NEAR(-MAX_STEERING_ANGLE, steering, 5.0f);
}

// ============================================================================
// 壁なしモードテスト
// ============================================================================

TEST_F(SteeringControllerTest, NoWalls_AlwaysZero) {
    WallDetection walls = createWallDetection(false, false, 0, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    for (int i = 1; i <= 10; i++) {
        ArduinoMock::setMillis(i * 40);
        float steering = controller.calculate(walls);
        EXPECT_FLOAT_EQ(0.0f, steering);
    }
}

// ============================================================================
// モード遷移テスト - 全組み合わせ
// ============================================================================

TEST_F(SteeringControllerTest, ModeTransition_BothToLeft_PIDReset) {
    WallDetection bothWalls = createWallDetection(true, true, 200, 400, 0, 0);
    WallDetection leftWall = createWallDetection(true, false, 300, 0, 0, 0);

    // 両壁モードで動作
    ArduinoMock::setMillis(0);
    controller.calculate(bothWalls);
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());

    // 数回動作させる
    for (int i = 1; i <= 5; i++) {
        ArduinoMock::setMillis(i * 40);
        controller.calculate(bothWalls);
    }

    // centeringPIDが使われている
    EXPECT_FALSE(controller.getCenteringPID().isFirstRun());

    // 左壁モードに遷移
    ArduinoMock::setMillis(240);
    controller.calculate(leftWall);

    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());
    // モード変更でcenteringPIDがリセットされる（左壁モードでは使わない）
    EXPECT_TRUE(controller.getCenteringPID().isFirstRun());
    // anglePIDはリセット後すぐにcompute()が呼ばれるためisFirstRun()はfalse
    EXPECT_FALSE(controller.getAnglePID().isFirstRun());
}

TEST_F(SteeringControllerTest, ModeTransition_BothToRight_PIDReset) {
    WallDetection bothWalls = createWallDetection(true, true, 300, 300, 0, 0);
    WallDetection rightWall = createWallDetection(false, true, 0, 300, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(bothWalls);
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());

    ArduinoMock::setMillis(40);
    controller.calculate(rightWall);
    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());
    EXPECT_TRUE(controller.getCenteringPID().isFirstRun());
}

TEST_F(SteeringControllerTest, ModeTransition_BothToNone_PIDReset) {
    WallDetection bothWalls = createWallDetection(true, true, 300, 300, 0, 0);
    WallDetection noWalls = createWallDetection(false, false, 0, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(bothWalls);

    ArduinoMock::setMillis(40);
    controller.calculate(noWalls);

    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());
    EXPECT_TRUE(controller.getCenteringPID().isFirstRun());
}

TEST_F(SteeringControllerTest, ModeTransition_LeftToBoth_PIDReset) {
    WallDetection leftWall = createWallDetection(true, false, 300, 0, 5, 0);
    WallDetection bothWalls = createWallDetection(true, true, 300, 300, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(leftWall);
    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());

    ArduinoMock::setMillis(40);
    controller.calculate(bothWalls);
    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_LeftToRight_PIDReset) {
    WallDetection leftWall = createWallDetection(true, false, 300, 0, 0, 0);
    WallDetection rightWall = createWallDetection(false, true, 0, 300, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(leftWall);
    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());

    ArduinoMock::setMillis(40);
    controller.calculate(rightWall);
    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_LeftToNone_PIDReset) {
    WallDetection leftWall = createWallDetection(true, false, 300, 0, 0, 0);
    WallDetection noWalls = createWallDetection(false, false, 0, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(leftWall);

    ArduinoMock::setMillis(40);
    controller.calculate(noWalls);

    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_RightToBoth_PIDReset) {
    WallDetection rightWall = createWallDetection(false, true, 0, 300, 0, 0);
    WallDetection bothWalls = createWallDetection(true, true, 300, 300, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(rightWall);

    ArduinoMock::setMillis(40);
    controller.calculate(bothWalls);

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_RightToLeft_PIDReset) {
    WallDetection rightWall = createWallDetection(false, true, 0, 300, 0, 0);
    WallDetection leftWall = createWallDetection(true, false, 300, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(rightWall);

    ArduinoMock::setMillis(40);
    controller.calculate(leftWall);

    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_RightToNone_PIDReset) {
    WallDetection rightWall = createWallDetection(false, true, 0, 300, 0, 0);
    WallDetection noWalls = createWallDetection(false, false, 0, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(rightWall);

    ArduinoMock::setMillis(40);
    controller.calculate(noWalls);

    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_NoneToBoth) {
    WallDetection noWalls = createWallDetection(false, false, 0, 0, 0, 0);
    WallDetection bothWalls = createWallDetection(true, true, 300, 300, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(noWalls);

    ArduinoMock::setMillis(40);
    controller.calculate(bothWalls);

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_NoneToLeft) {
    WallDetection noWalls = createWallDetection(false, false, 0, 0, 0, 0);
    WallDetection leftWall = createWallDetection(true, false, 300, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(noWalls);

    ArduinoMock::setMillis(40);
    controller.calculate(leftWall);

    EXPECT_EQ(MODE_LEFT_WALL, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_NoneToRight) {
    WallDetection noWalls = createWallDetection(false, false, 0, 0, 0, 0);
    WallDetection rightWall = createWallDetection(false, true, 0, 300, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(noWalls);

    ArduinoMock::setMillis(40);
    controller.calculate(rightWall);

    EXPECT_EQ(MODE_RIGHT_WALL, controller.getMode());
}

TEST_F(SteeringControllerTest, ModeTransition_SameMode_NoPIDReset) {
    WallDetection bothWalls1 = createWallDetection(true, true, 200, 400, 0, 0);
    WallDetection bothWalls2 = createWallDetection(true, true, 250, 350, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(bothWalls1);

    ArduinoMock::setMillis(40);
    controller.calculate(bothWalls1);

    // 同モード継続ならリセットされない
    EXPECT_FALSE(controller.getCenteringPID().isFirstRun());

    ArduinoMock::setMillis(80);
    controller.calculate(bothWalls2);

    EXPECT_FALSE(controller.getCenteringPID().isFirstRun());
}

// ============================================================================
// 出力クランプテスト
// ============================================================================

TEST_F(SteeringControllerTest, OutputClamp_UpperLimit) {
    // 極端な入力
    WallDetection walls = createWallDetection(true, true, 50, 800, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_FLOAT_EQ(MAX_STEERING_ANGLE, steering);
}

TEST_F(SteeringControllerTest, OutputClamp_LowerLimit) {
    WallDetection walls = createWallDetection(true, true, 800, 50, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_FLOAT_EQ(-MAX_STEERING_ANGLE, steering);
}

TEST_F(SteeringControllerTest, OutputClamp_LeftWall_WithSafety) {
    // 極端に近い壁
    WallDetection walls = createWallDetection(true, false, 50, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // 補正 = (300-50)*0.15 = 37.5 → 30にクランプ
    EXPECT_FLOAT_EQ(MAX_STEERING_ANGLE, steering);
}

// ============================================================================
// リセットテスト
// ============================================================================

TEST_F(SteeringControllerTest, Reset_ClearsAllState) {
    WallDetection walls = createWallDetection(true, true, 200, 400, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    controller.calculate(walls);

    EXPECT_EQ(MODE_BOTH_WALLS, controller.getMode());
    EXPECT_FALSE(controller.getCenteringPID().isFirstRun());

    controller.reset();

    EXPECT_EQ(MODE_NO_WALLS, controller.getMode());
    EXPECT_TRUE(controller.getCenteringPID().isFirstRun());
    EXPECT_TRUE(controller.getAnglePID().isFirstRun());
}

// ============================================================================
// 収束テスト
// ============================================================================

TEST_F(SteeringControllerTest, Convergence_BothWalls_GradualCentering) {
    // 左に偏った状態から徐々に中央へ
    float left_dist = 200.0f;
    float right_dist = 400.0f;

    ArduinoMock::setMillis(0);
    WallDetection walls = createWallDetection(true, true, left_dist, right_dist, 0, 0);
    controller.calculate(walls);

    std::vector<float> steering_history;
    for (int i = 1; i <= 15; i++) {
        ArduinoMock::setMillis(i * 40);

        // 徐々に中央に近づく
        float center = (left_dist + right_dist) / 2.0f;
        float diff = right_dist - left_dist;
        diff *= 0.9f;
        left_dist = center - diff / 2.0f;
        right_dist = center + diff / 2.0f;

        walls = createWallDetection(true, true, left_dist, right_dist, 0, 0);
        steering_history.push_back(controller.calculate(walls));
    }

    // 最後の出力は最初より小さい（収束）
    float first_abs = std::abs(steering_history.front());
    float last_abs = std::abs(steering_history.back());
    EXPECT_LT(last_abs, first_abs + 5.0f);
}

// ============================================================================
// 安定性テスト
// ============================================================================

TEST_F(SteeringControllerTest, Stability_OutputAlwaysBounded) {
    // 様々な入力でも出力は常に範囲内
    float distances[] = {50, 100, 200, 300, 500, 800, 1200};
    float angles[] = {-30, -15, -5, 0, 5, 15, 30};

    for (float ld : distances) {
        for (float rd : distances) {
            for (float la : angles) {
                for (float ra : angles) {
                    controller.reset();

                    WallDetection walls;
                    walls.left_valid = true;
                    walls.right_valid = true;
                    walls.left_distance = ld;
                    walls.right_distance = rd;
                    walls.left_angle = la;
                    walls.right_angle = ra;

                    ArduinoMock::setMillis(0);
                    controller.calculate(walls);

                    ArduinoMock::setMillis(40);
                    float steering = controller.calculate(walls);

                    EXPECT_GE(steering, -MAX_STEERING_ANGLE);
                    EXPECT_LE(steering, MAX_STEERING_ANGLE);
                    EXPECT_FALSE(std::isnan(steering));
                    EXPECT_FALSE(std::isinf(steering));
                }
            }
        }
    }
}

TEST_F(SteeringControllerTest, Stability_NoOscillation) {
    // 一定の入力で出力が安定
    WallDetection walls = createWallDetection(true, true, 250, 350, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    std::vector<float> outputs;
    for (int i = 1; i <= 20; i++) {
        ArduinoMock::setMillis(i * 40);
        outputs.push_back(controller.calculate(walls));
    }

    // 最後の5回の変化が小さい
    float max_change = 0.0f;
    for (size_t i = 15; i < 20; i++) {
        float change = std::abs(outputs[i] - outputs[i-1]);
        max_change = std::max(max_change, change);
    }

    EXPECT_LT(max_change, 3.0f);  // 急激な変化なし
}

// ============================================================================
// エッジケーステスト
// ============================================================================

TEST_F(SteeringControllerTest, EdgeCase_ZeroDistances) {
    // 距離0（センサー異常）
    WallDetection walls = createWallDetection(true, true, 0, 0, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_FALSE(std::isnan(steering));
    EXPECT_FALSE(std::isinf(steering));
}

TEST_F(SteeringControllerTest, EdgeCase_VeryLargeDistances) {
    WallDetection walls = createWallDetection(true, true, 10000, 10000, 0, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    EXPECT_FALSE(std::isnan(steering));
    EXPECT_GE(steering, -MAX_STEERING_ANGLE);
    EXPECT_LE(steering, MAX_STEERING_ANGLE);
}

TEST_F(SteeringControllerTest, EdgeCase_LargeAngle) {
    // 極端な角度
    WallDetection walls = createWallDetection(true, false, 400, 0, 90, 0);

    ArduinoMock::setMillis(0);
    controller.calculate(walls);

    ArduinoMock::setMillis(40);
    float steering = controller.calculate(walls);

    // クランプされる
    EXPECT_GE(steering, -MAX_STEERING_ANGLE);
    EXPECT_LE(steering, MAX_STEERING_ANGLE);
}

TEST_F(SteeringControllerTest, EdgeCase_FirstCallReturnsZero) {
    // 初回呼び出しはPIDが0を返すため、安全距離補正のみ
    WallDetection walls = createWallDetection(true, true, 200, 400, 0, 0);

    ArduinoMock::setMillis(0);
    float steering = controller.calculate(walls);

    EXPECT_FLOAT_EQ(0.0f, steering);  // 初回は0
}

