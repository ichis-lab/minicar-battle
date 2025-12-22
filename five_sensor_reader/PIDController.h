/*
 * PIDController.h
 *
 * 汎用PIDコントローラ（宣言）
 * Generic PID Controller
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

// PIDゲイン構造体
struct PIDGains {
    float Kp;           // 比例ゲイン / Proportional gain
    float Ki;           // 積分ゲイン / Integral gain
    float Kd;           // 微分ゲイン / Derivative gain
};

// PID状態構造体
struct PIDState {
    float prev_error;           // 前回の誤差 / Previous error
    float integral;             // 積分値 / Integral accumulator
    float filtered_derivative;  // フィルタ済み微分値 / Filtered derivative
    unsigned long last_time;    // 前回の計算時刻 / Last calculation time
};

// PID設定構造体
struct PIDConfig {
    float output_min;    // 出力下限 / Output minimum
    float output_max;    // 出力上限 / Output maximum
    float integral_min;  // 積分下限（アンチワインドアップ）/ Integral min (anti-windup)
    float integral_max;  // 積分上限（アンチワインドアップ）/ Integral max (anti-windup)
    float deadband;      // 不感帯 / Deadband threshold
    float filter_alpha;  // 微分フィルタ係数 / Derivative filter coefficient
};

class PIDController {
private:
    PIDGains gains;
    PIDState state;
    PIDConfig config;
    bool first_run;

public:
    PIDController();

    // 初期化 / Initialize
    void begin(float Kp, float Ki, float Kd);

    // 設定 / Configuration
    void setGains(float Kp, float Ki, float Kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setDeadband(float deadband);
    void setFilterAlpha(float alpha);

    // 計算 / Calculation
    float compute(float setpoint, float measured);

    // リセット / Reset
    void reset();

    // デバッグ用 / For debugging
    float getProportional() const;
    float getIntegral() const;
    float getDerivative() const;
    float getError() const;
};

#endif // PID_CONTROLLER_H
