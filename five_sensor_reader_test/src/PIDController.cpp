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
#include <cmath>

PIDController::PIDController() {
    _gains = {0.0, 0.0, 0.0};
    _state = {0.0, 0.0, 0.0, 0};  // prev_error, integral, filtered_derivative, last_time
    _config = {-180.0, 180.0, -100.0, 100.0, 0.0, DERIVATIVE_FILTER_ALPHA};
    _firstRun = true;
}

void PIDController::begin(float Kp, float Ki, float Kd) {
    setGains(Kp, Ki, Kd);
    reset();
}

void PIDController::setGains(float Kp, float Ki, float Kd) {
    _gains.Kp = Kp;
    _gains.Ki = Ki;
    _gains.Kd = Kd;
}

void PIDController::setOutputLimits(float min, float max) {
    _config.output_min = min;
    _config.output_max = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    _config.integral_min = min;
    _config.integral_max = max;
}

void PIDController::setDeadband(float deadband) {
    _config.deadband = deadband;
}

void PIDController::setFilterAlpha(float alpha) {
    _config.filter_alpha = constrain(alpha, 0.0f, 1.0f);
}

float PIDController::compute(float setpoint, float measured) {
    unsigned long now = millis();

    // 初回実行時の処理 / First run handling
    if (_firstRun) {
        _state.last_time = now;
        _state.prev_error = setpoint - measured;
        _state.filtered_derivative = 0.0;
        _firstRun = false;
        return 0.0;
    }

    // 時間差の計算 / Calculate time delta
    float dt = (now - _state.last_time) / 1000.0f;  // 秒に変換 / Convert to seconds
    if (dt <= 0.0f) dt = 0.001f;  // ゼロ除算防止 / Prevent division by zero

    // 誤差の計算 / Calculate error
    float error = setpoint - measured;

    // 不感帯の適用 / Apply deadband
    if (std::abs(error) < _config.deadband) {
        error = 0.0f;
    }

    // P項 / Proportional term
    float P = _gains.Kp * error;

    // I項（アンチワインドアップ付き）/ Integral term with anti-windup
    _state.integral += error * dt;
    _state.integral = constrain(_state.integral, _config.integral_min, _config.integral_max);
    float I = _gains.Ki * _state.integral;

    // D項（ローパスフィルタ付き）/ Derivative term with low-pass filter
    float raw_derivative = (error - _state.prev_error) / dt;

    // 一次ローパスフィルタ: filtered = alpha * raw + (1 - alpha) * prev_filtered
    _state.filtered_derivative = _config.filter_alpha * raw_derivative
                               + (1.0f - _config.filter_alpha) * _state.filtered_derivative;

    float D = _gains.Kd * _state.filtered_derivative;

    // 出力の計算 / Calculate output
    float output = P + I + D;

    // 出力制限 / Output limiting
    output = constrain(output, _config.output_min, _config.output_max);

    // 状態の更新 / Update state
    _state.prev_error = error;
    _state.last_time = now;

    return output;
}

void PIDController::reset() {
    _state.prev_error = 0.0;
    _state.integral = 0.0;
    _state.filtered_derivative = 0.0;
    _state.last_time = millis();
    _firstRun = true;
}

float PIDController::getProportional() const {
    return _gains.Kp * _state.prev_error;
}

float PIDController::getIntegral() const {
    return _gains.Ki * _state.integral;
}

float PIDController::getDerivative() const {
    return _gains.Kd * _state.filtered_derivative;
}

float PIDController::getError() const {
    return _state.prev_error;
}
