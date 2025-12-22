/*
 * PIDController.cpp
 *
 * 汎用PIDコントローラ（実装）
 * Generic PID Controller Implementation
 */

#include "PIDController.h"

PIDController::PIDController() {
    gains = {0.0, 0.0, 0.0};
    state = {0.0, 0.0, 0};
    config = {-180.0, 180.0, -100.0, 100.0, 0.0};
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

float PIDController::compute(float setpoint, float measured) {
    unsigned long now = millis();

    // 初回実行時の処理 / First run handling
    if (first_run) {
        state.last_time = now;
        state.prev_error = setpoint - measured;
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

    // D項 / Derivative term
    float derivative = (error - state.prev_error) / dt;
    float D = gains.Kd * derivative;

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
    return 0.0;  // 簡略化（現在のD値は保存していない）
}

float PIDController::getError() const {
    return state.prev_error;
}
