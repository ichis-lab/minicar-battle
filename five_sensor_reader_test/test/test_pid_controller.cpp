/*
 * test_pid_controller.cpp
 *
 * PIDController単体テスト
 *
 * 仕様:
 * - error = setpoint - measured
 * - P = Kp * error
 * - I = Ki * integral (integral += error * dt)
 * - D = Kd * filtered_derivative
 * - filtered_derivative = alpha * raw + (1-alpha) * prev
 * - output = clamp(P + I + D, min, max)
 * - 不感帯: |error| < deadband → error = 0
 */

#include "test_common.h"

using namespace TestConstants;

class PIDControllerTest : public BaseTestFixture {
protected:
    PIDController pid;
};

// ============================================================================
// 基本動作テスト
// ============================================================================

TEST_F(PIDControllerTest, Constructor_InitializesCorrectly) {
    EXPECT_TRUE(pid.isFirstRun());
    EXPECT_FLOAT_EQ(0.0f, pid.getIntegralValue());
    EXPECT_FLOAT_EQ(0.0f, pid.getFilteredDerivative());
}

TEST_F(PIDControllerTest, Begin_SetsGainsAndResets) {
    pid.begin(1.0f, 0.5f, 0.1f);
    EXPECT_TRUE(pid.isFirstRun());
}

TEST_F(PIDControllerTest, Reset_ClearsAllState) {
    pid.begin(1.0f, 0.1f, 0.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);  // 初回

    ArduinoMock::setMillis(40);
    pid.compute(100.0f, 0.0f);  // 積分蓄積

    EXPECT_NE(0.0f, pid.getIntegralValue());
    EXPECT_FALSE(pid.isFirstRun());

    pid.reset();

    EXPECT_TRUE(pid.isFirstRun());
    EXPECT_FLOAT_EQ(0.0f, pid.getIntegralValue());
    EXPECT_FLOAT_EQ(0.0f, pid.getFilteredDerivative());
}

// ============================================================================
// P項（比例制御）テスト - 手計算による期待値検証
// ============================================================================

TEST_F(PIDControllerTest, P_PositiveError_ExactCalculation) {
    // Kp=1.0, error=50 → P = 1.0 * 50 = 50
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 50.0f);  // 初回

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 50.0f);

    EXPECT_FLOAT_EQ(50.0f, output);
}

TEST_F(PIDControllerTest, P_NegativeError_ExactCalculation) {
    // Kp=1.0, error=-50 → P = 1.0 * (-50) = -50
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(50.0f, 100.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(50.0f, 100.0f);

    EXPECT_FLOAT_EQ(-50.0f, output);
}

TEST_F(PIDControllerTest, P_ZeroError_ZeroOutput) {
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 100.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 100.0f);

    EXPECT_FLOAT_EQ(0.0f, output);
}

TEST_F(PIDControllerTest, P_GainMultiplier_ExactCalculation) {
    // Kp=0.5, error=100 → P = 0.5 * 100 = 50
    pid.begin(0.5f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 0.0f);

    EXPECT_FLOAT_EQ(50.0f, output);
}

TEST_F(PIDControllerTest, P_FractionalGain_ExactCalculation) {
    // Kp=0.08, error=200 → P = 0.08 * 200 = 16
    pid.begin(0.08f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(200.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(200.0f, 0.0f);

    EXPECT_FLOAT_EQ(16.0f, output);
}

// ============================================================================
// I項（積分制御）テスト - 手計算による期待値検証
// ============================================================================

TEST_F(PIDControllerTest, I_SingleStep_ExactCalculation) {
    // Ki=1.0, error=100, dt=0.04s
    // integral = 100 * 0.04 = 4.0
    // I = 1.0 * 4.0 = 4.0
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);  // 初回

    ArduinoMock::setMillis(40);  // dt = 40ms = 0.04s
    float output = pid.compute(100.0f, 0.0f);

    EXPECT_FLOAT_EQ(4.0f, pid.getIntegralValue());
    EXPECT_FLOAT_EQ(4.0f, output);
}

TEST_F(PIDControllerTest, I_MultipleSteps_AccumulationExact) {
    // Ki=1.0, error=100, 5回 × dt=0.04s
    // integral = 100 * 0.04 * 5 = 20.0
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    for (int i = 1; i <= 5; i++) {
        ArduinoMock::setMillis(i * 40);
        pid.compute(100.0f, 0.0f);
    }

    EXPECT_FLOAT_EQ(20.0f, pid.getIntegralValue());
}

TEST_F(PIDControllerTest, I_VariableError_AccumulationCorrect) {
    // dt=0.04s, errors: 100, 50, 25
    // integral = 100*0.04 + 50*0.04 + 25*0.04 = 4 + 2 + 1 = 7.0
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);  // 初回

    ArduinoMock::setMillis(40);
    pid.compute(100.0f, 0.0f);  // integral += 100 * 0.04 = 4

    ArduinoMock::setMillis(80);
    pid.compute(50.0f, 0.0f);   // integral += 50 * 0.04 = 2

    ArduinoMock::setMillis(120);
    pid.compute(25.0f, 0.0f);   // integral += 25 * 0.04 = 1

    EXPECT_FLOAT_EQ(7.0f, pid.getIntegralValue());
}

TEST_F(PIDControllerTest, I_AntiWindup_UpperLimit_Exact) {
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-50.0f, 50.0f);  // 積分を±50に制限

    ArduinoMock::setMillis(0);
    pid.compute(1000.0f, 0.0f);  // 大きな誤差

    // 1000 * 0.04 = 40 → 3回で120だが50にクランプ
    for (int i = 1; i <= 10; i++) {
        ArduinoMock::setMillis(i * 40);
        pid.compute(1000.0f, 0.0f);
    }

    EXPECT_FLOAT_EQ(50.0f, pid.getIntegralValue());
}

TEST_F(PIDControllerTest, I_AntiWindup_LowerLimit_Exact) {
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-50.0f, 50.0f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 1000.0f);  // error = -1000

    for (int i = 1; i <= 10; i++) {
        ArduinoMock::setMillis(i * 40);
        pid.compute(0.0f, 1000.0f);
    }

    EXPECT_FLOAT_EQ(-50.0f, pid.getIntegralValue());
}

// ============================================================================
// D項（微分制御）テスト - 手計算による期待値検証
// ============================================================================

TEST_F(PIDControllerTest, D_StepChange_NoFilter_ExactCalculation) {
    // Kd=1.0, alpha=1.0（フィルタなし）
    // 初回: error=0, 2回目: error=100
    // raw_derivative = (100 - 0) / 1.0 = 100
    // filtered = 1.0 * 100 + 0 * 0 = 100
    // D = 1.0 * 100 = 100
    pid.begin(0.0f, 0.0f, 1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setFilterAlpha(1.0f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 0.0f);  // error=0

    ArduinoMock::setMillis(1000);  // dt = 1.0s
    float output = pid.compute(100.0f, 0.0f);  // error=100

    EXPECT_FLOAT_EQ(100.0f, pid.getFilteredDerivative());
    EXPECT_FLOAT_EQ(100.0f, output);
}

TEST_F(PIDControllerTest, D_StepChange_WithFilter_ExactCalculation) {
    // Kd=1.0, alpha=0.5
    // 初回: error=0, filtered_derivative=0
    // 2回目: error=100, raw_derivative = 100/1.0 = 100
    // filtered = 0.5 * 100 + 0.5 * 0 = 50
    // D = 1.0 * 50 = 50
    pid.begin(0.0f, 0.0f, 1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setFilterAlpha(0.5f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 0.0f);

    ArduinoMock::setMillis(1000);
    float output = pid.compute(100.0f, 0.0f);

    EXPECT_FLOAT_EQ(50.0f, pid.getFilteredDerivative());
    EXPECT_FLOAT_EQ(50.0f, output);
}

TEST_F(PIDControllerTest, D_FilterAlphaZero_NoChange) {
    // alpha=0: filtered = 0 * raw + 1 * prev = prev
    pid.begin(0.0f, 0.0f, 1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setFilterAlpha(0.0f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 0.0f);  // filtered=0

    ArduinoMock::setMillis(1000);
    pid.compute(100.0f, 0.0f);  // raw=100, filtered=0*100+1*0=0

    EXPECT_FLOAT_EQ(0.0f, pid.getFilteredDerivative());
}

TEST_F(PIDControllerTest, D_DecreasingError_NegativeDerivative) {
    pid.begin(0.0f, 0.0f, 1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setFilterAlpha(1.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);  // error=100

    ArduinoMock::setMillis(1000);
    float output = pid.compute(0.0f, 0.0f);  // error=0

    // raw_derivative = (0 - 100) / 1.0 = -100
    EXPECT_FLOAT_EQ(-100.0f, output);
}

TEST_F(PIDControllerTest, D_FilterConvergence_MultipleSteps) {
    // alpha=0.3で複数回
    // filtered[n] = 0.3 * raw + 0.7 * filtered[n-1]
    pid.begin(0.0f, 0.0f, 1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setFilterAlpha(0.3f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 0.0f);

    ArduinoMock::setMillis(1000);
    pid.compute(100.0f, 0.0f);
    float f1 = pid.getFilteredDerivative();
    // f1 = 0.3 * 100 + 0.7 * 0 = 30

    ArduinoMock::setMillis(2000);
    pid.compute(100.0f, 0.0f);  // error変化なし、raw_derivative=0
    float f2 = pid.getFilteredDerivative();
    // f2 = 0.3 * 0 + 0.7 * 30 = 21

    EXPECT_NEAR(30.0f, f1, 0.01f);
    EXPECT_NEAR(21.0f, f2, 0.01f);
}

// ============================================================================
// 不感帯（デッドバンド）テスト - 境界値
// ============================================================================

TEST_F(PIDControllerTest, Deadband_InsideDeadband_ZeroError) {
    // |error| < deadband → error = 0
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setDeadband(10.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 97.0f);  // error=3

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 97.0f);

    EXPECT_FLOAT_EQ(0.0f, output);
}

TEST_F(PIDControllerTest, Deadband_OnBoundary_NotZero) {
    // |error| = deadband → 不感帯外（<で比較）
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setDeadband(10.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 90.0f);  // error=10

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 90.0f);

    EXPECT_FLOAT_EQ(10.0f, output);  // 境界は不感帯外
}

TEST_F(PIDControllerTest, Deadband_JustInside_ZeroOutput) {
    // error = 9.99 (< 10) → 不感帯内
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setDeadband(10.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 90.01f);  // error ≈ 9.99

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 90.01f);

    EXPECT_FLOAT_EQ(0.0f, output);
}

TEST_F(PIDControllerTest, Deadband_NegativeError_ZeroOutput) {
    // |error| = |-5| = 5 < 10 → 不感帯内
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setDeadband(10.0f);

    ArduinoMock::setMillis(0);
    pid.compute(95.0f, 100.0f);  // error=-5

    ArduinoMock::setMillis(40);
    float output = pid.compute(95.0f, 100.0f);

    EXPECT_FLOAT_EQ(0.0f, output);
}

TEST_F(PIDControllerTest, Deadband_DoesNotAffectIntegral) {
    // 不感帯内だと積分も0になる（error=0扱い）
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);
    pid.setDeadband(10.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 95.0f);  // error=5（不感帯内）

    ArduinoMock::setMillis(40);
    pid.compute(100.0f, 95.0f);

    EXPECT_FLOAT_EQ(0.0f, pid.getIntegralValue());
}

// ============================================================================
// 出力制限テスト - 境界値
// ============================================================================

TEST_F(PIDControllerTest, OutputLimits_UpperClamp_Exact) {
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-30.0f, 30.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 0.0f);

    EXPECT_FLOAT_EQ(30.0f, output);
}

TEST_F(PIDControllerTest, OutputLimits_LowerClamp_Exact) {
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-30.0f, 30.0f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 100.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(0.0f, 100.0f);

    EXPECT_FLOAT_EQ(-30.0f, output);
}

TEST_F(PIDControllerTest, OutputLimits_OnUpperBoundary_NoClamp) {
    // P = 30（ちょうど上限）→ クランプされない
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-30.0f, 30.0f);

    ArduinoMock::setMillis(0);
    pid.compute(30.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(30.0f, 0.0f);

    EXPECT_FLOAT_EQ(30.0f, output);
}

TEST_F(PIDControllerTest, OutputLimits_JustOverLimit_Clamped) {
    // P = 30.1 → 30にクランプ
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-30.0f, 30.0f);

    ArduinoMock::setMillis(0);
    pid.compute(30.1f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(30.1f, 0.0f);

    EXPECT_FLOAT_EQ(30.0f, output);
}

TEST_F(PIDControllerTest, OutputLimits_Passthrough) {
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-30.0f, 30.0f);

    ArduinoMock::setMillis(0);
    pid.compute(15.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(15.0f, 0.0f);

    EXPECT_FLOAT_EQ(15.0f, output);
}

// ============================================================================
// 初回実行・時間管理テスト
// ============================================================================

TEST_F(PIDControllerTest, FirstRun_ReturnsZero) {
    pid.begin(1.0f, 1.0f, 1.0f);

    ArduinoMock::setMillis(0);
    float output = pid.compute(100.0f, 0.0f);

    EXPECT_FLOAT_EQ(0.0f, output);
    EXPECT_FALSE(pid.isFirstRun());  // 初回実行後はfalse
}

TEST_F(PIDControllerTest, FirstRun_InitializesPreviousError) {
    pid.begin(0.0f, 0.0f, 1.0f);
    pid.setFilterAlpha(1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(50.0f, 0.0f);  // prev_error = 50

    ArduinoMock::setMillis(1000);
    float output = pid.compute(100.0f, 0.0f);  // error=100

    // raw_derivative = (100 - 50) / 1.0 = 50
    EXPECT_FLOAT_EQ(50.0f, output);
}

TEST_F(PIDControllerTest, SameTime_UsesMinimumDt) {
    // dt=0 → dt=0.001に補正
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(100);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(100);  // 同時刻
    float output = pid.compute(100.0f, 0.0f);

    // integral += 100 * 0.001 = 0.1
    EXPECT_FALSE(std::isnan(output));
    EXPECT_FALSE(std::isinf(output));
    EXPECT_NEAR(0.1f, pid.getIntegralValue(), 0.001f);
}

// ============================================================================
// PID複合テスト - 手計算による検証
// ============================================================================

TEST_F(PIDControllerTest, PID_AllTerms_ExactCalculation) {
    // Kp=0.1, Ki=0.5, Kd=0.2, alpha=1.0
    // dt=0.04s, error=100
    // 2回目: P=10, I=0.5*4=2, D=0.2*0=0 → output=12
    pid.begin(0.1f, 0.5f, 0.2f);
    pid.setOutputLimits(-100.0f, 100.0f);
    pid.setIntegralLimits(-100.0f, 100.0f);
    pid.setFilterAlpha(1.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);  // 初回, prev_error=100

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 0.0f);

    // P = 0.1 * 100 = 10
    // I = 0.5 * (100 * 0.04) = 0.5 * 4 = 2
    // D = 0.2 * ((100 - 100) / 0.04) = 0
    // output = 10 + 2 + 0 = 12
    EXPECT_NEAR(12.0f, output, 0.01f);
}

TEST_F(PIDControllerTest, PID_ThirdStep_ExactCalculation) {
    // 3回目の計算を検証
    pid.begin(0.1f, 0.5f, 0.2f);
    pid.setOutputLimits(-100.0f, 100.0f);
    pid.setIntegralLimits(-100.0f, 100.0f);
    pid.setFilterAlpha(1.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);  // 初回

    ArduinoMock::setMillis(40);
    pid.compute(100.0f, 0.0f);  // 2回目: integral=4

    ArduinoMock::setMillis(80);
    float output = pid.compute(100.0f, 0.0f);  // 3回目

    // P = 0.1 * 100 = 10
    // I = 0.5 * (4 + 4) = 0.5 * 8 = 4
    // D = 0.2 * 0 = 0
    // output = 10 + 4 + 0 = 14
    EXPECT_NEAR(14.0f, output, 0.01f);
}

// ============================================================================
// ステップ応答テスト
// ============================================================================

TEST_F(PIDControllerTest, StepResponse_ProportionalOnly_Immediate) {
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-100.0f, 100.0f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 0.0f);

    // ステップ入力（0→50）
    ArduinoMock::setMillis(40);
    float output = pid.compute(50.0f, 0.0f);

    // P制御のみなので即座に反応
    EXPECT_FLOAT_EQ(50.0f, output);
}

TEST_F(PIDControllerTest, StepResponse_IntegralOnly_Ramp) {
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    std::vector<float> outputs;
    for (int i = 1; i <= 5; i++) {
        ArduinoMock::setMillis(i * 40);
        outputs.push_back(pid.compute(100.0f, 0.0f));
    }

    // I制御は線形に増加: 4, 8, 12, 16, 20
    EXPECT_NEAR(4.0f, outputs[0], 0.01f);
    EXPECT_NEAR(8.0f, outputs[1], 0.01f);
    EXPECT_NEAR(12.0f, outputs[2], 0.01f);
    EXPECT_NEAR(16.0f, outputs[3], 0.01f);
    EXPECT_NEAR(20.0f, outputs[4], 0.01f);
}

TEST_F(PIDControllerTest, StepResponse_DerivativeOnly_ImpulseAndDecay) {
    // D制御はステップ変化時にスパイク、その後0
    pid.begin(0.0f, 0.0f, 1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setFilterAlpha(1.0f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 0.0f);  // error=0

    ArduinoMock::setMillis(1000);
    float spike = pid.compute(100.0f, 0.0f);  // ステップ変化

    ArduinoMock::setMillis(2000);
    float decay = pid.compute(100.0f, 0.0f);  // 変化なし

    EXPECT_FLOAT_EQ(100.0f, spike);  // スパイク
    EXPECT_FLOAT_EQ(0.0f, decay);    // 変化なしなので0
}

// ============================================================================
// 安定性テスト
// ============================================================================

TEST_F(PIDControllerTest, Stability_OutputBounded) {
    // どんな入力でも出力が制限内に収まる
    pid.begin(1.0f, 1.0f, 1.0f);
    pid.setOutputLimits(-30.0f, 30.0f);
    pid.setIntegralLimits(-50.0f, 50.0f);

    ArduinoMock::setMillis(0);
    pid.compute(10000.0f, -10000.0f);  // 極端な入力

    for (int i = 1; i <= 100; i++) {
        ArduinoMock::setMillis(i * 40);
        float output = pid.compute(10000.0f, -10000.0f);
        EXPECT_GE(output, -30.0f);
        EXPECT_LE(output, 30.0f);
    }
}

TEST_F(PIDControllerTest, Stability_NoNaNOrInf) {
    pid.begin(1.0f, 1.0f, 1.0f);
    pid.setOutputLimits(-100.0f, 100.0f);
    pid.setIntegralLimits(-100.0f, 100.0f);
    pid.setFilterAlpha(0.3f);

    ArduinoMock::setMillis(0);
    pid.compute(0.0f, 0.0f);

    // ランダムな入力パターン
    float setpoints[] = {100, -50, 200, 0, -100, 150, 80, -30, 0, 100};
    float measureds[] = {0, 100, -50, 200, 0, -100, 150, 80, 200, 0};

    for (int i = 0; i < 10; i++) {
        ArduinoMock::advanceMillis(40);
        float output = pid.compute(setpoints[i], measureds[i]);
        EXPECT_FALSE(std::isnan(output));
        EXPECT_FALSE(std::isinf(output));
    }
}

// ============================================================================
// 収束テスト
// ============================================================================

TEST_F(PIDControllerTest, Convergence_ErrorDecreasing_OutputDecreasing) {
    // 誤差が減少するとP出力も減少
    pid.begin(1.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-100.0f, 100.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    float prev_output = 1000.0f;
    float errors[] = {100.0f, 80.0f, 60.0f, 40.0f, 20.0f, 10.0f, 5.0f};

    for (int i = 0; i < 7; i++) {
        ArduinoMock::advanceMillis(40);
        float measured = 100.0f - errors[i];
        float output = pid.compute(100.0f, measured);

        // P制御なので出力は単調減少
        EXPECT_LT(std::abs(output), std::abs(prev_output) + 1.0f);
        prev_output = output;
    }
}

TEST_F(PIDControllerTest, Convergence_PIController_EventualZeroOutput) {
    // PI制御で誤差0になると、積分項も減衰して最終的に出力が小さくなる
    pid.begin(0.5f, 0.1f, 0.0f);
    pid.setOutputLimits(-100.0f, 100.0f);
    pid.setIntegralLimits(-50.0f, 50.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    // 誤差が徐々に0に
    float errors[] = {100, 80, 60, 40, 20, 10, 5, 2, 0, 0, 0};
    for (int i = 0; i < 11; i++) {
        ArduinoMock::advanceMillis(40);
        float measured = 100.0f - errors[i];
        pid.compute(100.0f, measured);
    }

    // 最後の数回は誤差0
    ArduinoMock::advanceMillis(40);
    float final = pid.compute(100.0f, 100.0f);

    // P=0だが、積分項が残っているので完全に0ではない
    // ただし出力は小さくなっているはず
    EXPECT_LT(std::abs(final), 50.0f);
}

// ============================================================================
// 異常系テスト
// ============================================================================

TEST_F(PIDControllerTest, Abnormal_ZeroGains_ZeroOutput) {
    pid.begin(0.0f, 0.0f, 0.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 0.0f);

    EXPECT_FLOAT_EQ(0.0f, output);
}

TEST_F(PIDControllerTest, Abnormal_VerySmallGains_SmallOutput) {
    pid.begin(0.001f, 0.001f, 0.001f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);
    pid.setFilterAlpha(1.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(100.0f, 0.0f);

    // P = 0.001 * 100 = 0.1
    // I = 0.001 * 4 = 0.004
    // D = 0.001 * 0 = 0
    EXPECT_NEAR(0.104f, output, 0.01f);
}

TEST_F(PIDControllerTest, Abnormal_VeryLargeDt_IntegralAccumulation) {
    pid.begin(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);
    pid.setIntegralLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(10000);  // 10秒
    float output = pid.compute(100.0f, 0.0f);

    // integral = 100 * 10 = 1000（上限でクランプ）
    EXPECT_FLOAT_EQ(1000.0f, output);
}

TEST_F(PIDControllerTest, Abnormal_DefaultOutputLimits) {
    // デフォルトは-180, 180
    pid.begin(1.0f, 0.0f, 0.0f);

    ArduinoMock::setMillis(0);
    pid.compute(500.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output = pid.compute(500.0f, 0.0f);

    EXPECT_FLOAT_EQ(180.0f, output);
}

// ============================================================================
// 設定変更テスト
// ============================================================================

TEST_F(PIDControllerTest, SetGains_UpdatesCorrectly) {
    pid.begin(1.0f, 1.0f, 1.0f);
    pid.setOutputLimits(-1000.0f, 1000.0f);

    ArduinoMock::setMillis(0);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(40);
    float output1 = pid.compute(100.0f, 0.0f);

    // ゲイン変更
    pid.setGains(2.0f, 0.0f, 0.0f);
    pid.reset();

    ArduinoMock::setMillis(80);
    pid.compute(100.0f, 0.0f);

    ArduinoMock::setMillis(120);
    float output2 = pid.compute(100.0f, 0.0f);

    // Kp=2.0 → P = 200
    EXPECT_FLOAT_EQ(200.0f, output2);
}

