/*
 * PIDController.cpp
 *
 * 汎用PIDコントローラ（実装）
 * Generic PID Controller Implementation
 */

#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd,
                             float i_limit, float out_min, float out_max)
    : Kp(kp), Ki(ki), Kd(kd),
      prev_error(0.0), integral(0.0), prev_time(0),
      integral_limit(i_limit), output_min(out_min), output_max(out_max) {
}

float PIDController::calculate(float error) {
    unsigned long current_time = millis();

    // 初回呼び出し時はdt=0として微分項をスキップ
    float dt = 0.0;
    if (prev_time > 0) {
        dt = (current_time - prev_time) / 1000.0;  // ミリ秒→秒
    }

    // 比例項 / Proportional term
    float P = Kp * error;

    // 積分項 / Integral term
    if (dt > 0 && Ki != 0.0) {
        integral += error * dt;
        // 積分値のアンチワインドアップ / Anti-windup
        if (integral > integral_limit) {
            integral = integral_limit;
        } else if (integral < -integral_limit) {
            integral = -integral_limit;
        }
    }
    float I = Ki * integral;

    // 微分項 / Derivative term
    float D = 0.0;
    if (dt > 0 && Kd != 0.0) {
        float derivative = (error - prev_error) / dt;
        D = Kd * derivative;
    }

    // 出力計算 / Calculate output
    float output = P + I + D;

    // 出力制限 / Clamp output
    if (output > output_max) {
        output = output_max;
    } else if (output < output_min) {
        output = output_min;
    }

    // 状態更新 / Update state
    prev_error = error;
    prev_time = current_time;

    return output;
}

void PIDController::reset() {
    prev_error = 0.0;
    integral = 0.0;
    prev_time = 0;
}

void PIDController::setGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}
