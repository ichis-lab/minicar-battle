/*
 * PIDController.cpp
 *
 * 汎用PIDコントローラ（実装）
 * Generic PID Controller Implementation
 *
 * 機能:
 * - アンチワインドアップ（積分値制限）
 * - 不感帯（微小誤差での振動防止）
 * - 微分フィルタ（センサーノイズ抑制）
 */

#include "PIDController.h"
#include "Config.h"

PIDController::PIDController() {
    gains = {0.0, 0.0, 0.0};
    state = {0.0, 0.0, 0.0, 0};  // prev_error, integral, filtered_derivative, last_time
    config = {-180.0, 180.0, -100.0, 100.0, 0.0, DERIVATIVE_FILTER_ALPHA};
    first_run = true;
}

void PIDController::begin(float Kp, float Ki, float Kd) {
    setGains(Kp, Ki, Kd);
    reset();
}

void PIDController::setGains(float Kp, float Ki, float Kd) {
    gains.Kp = Kp;
    gains.Ki = Ki;
    gains.Kd = Kd;
}

void PIDController::setOutputLimits(float min, float max) {
    config.output_min = min;
    config.output_max = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    config.integral_min = min;
    config.integral_max = max;
}

void PIDController::setDeadband(float deadband) {
    config.deadband = deadband;
}

void PIDController::setFilterAlpha(float alpha) {
    config.filter_alpha = constrain(alpha, 0.0, 1.0);
}

float PIDController::compute(float setpoint, float measured) {
    unsigned long now = millis();

    // 初回実行時の処理 / First run handling
    if (first_run) {
        state.last_time = now;
        state.prev_error = setpoint - measured;
        state.filtered_derivative = 0.0;
        first_run = false;
        return 0.0;
    }

    // 時間差の計算 / Calculate time delta
    float dt = (now - state.last_time) / 1000.0;  // 秒に変換 / Convert to seconds
    if (dt <= 0.0) dt = 0.001;  // ゼロ除算防止 / Prevent division by zero

    // 誤差の計算 / Calculate error
    float error = setpoint - measured;

    // 不感帯の適用 / Apply deadband
    if (abs(error) < config.deadband) {
        error = 0.0;
    }

    // P項 / Proportional term
    float P = gains.Kp * error;

    // I項（アンチワインドアップ付き）/ Integral term with anti-windup
    state.integral += error * dt;
    state.integral = constrain(state.integral, config.integral_min, config.integral_max);
    float I = gains.Ki * state.integral;

    // D項（ローパスフィルタ付き）/ Derivative term with low-pass filter
    float raw_derivative = (error - state.prev_error) / dt;

    // 一次ローパスフィルタ: filtered = alpha * raw + (1 - alpha) * prev_filtered
    state.filtered_derivative = config.filter_alpha * raw_derivative
                               + (1.0 - config.filter_alpha) * state.filtered_derivative;

    float D = gains.Kd * state.filtered_derivative;

    // 出力の計算 / Calculate output
    float output = P + I + D;

    // 出力制限 / Output limiting
    output = constrain(output, config.output_min, config.output_max);

    // 状態の更新 / Update state
    state.prev_error = error;
    state.last_time = now;

    return output;
}

void PIDController::reset() {
    state.prev_error = 0.0;
    state.integral = 0.0;
    state.filtered_derivative = 0.0;
    state.last_time = millis();
    first_run = true;
}

float PIDController::getProportional() const {
    return gains.Kp * state.prev_error;
}

float PIDController::getIntegral() const {
    return gains.Ki * state.integral;
}

float PIDController::getDerivative() const {
    return gains.Kd * state.filtered_derivative;
}

float PIDController::getError() const {
    return state.prev_error;
}
