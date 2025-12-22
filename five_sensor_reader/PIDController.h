/*
 * PIDController.h
 *
 * 汎用PIDコントローラ（宣言）
 * Generic PID Controller
 *
 * 任意の制御対象に使用可能な汎用PID実装
 * Reusable PID implementation for any control target
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
private:
    float Kp;           // 比例ゲイン / Proportional gain
    float Ki;           // 積分ゲイン / Integral gain
    float Kd;           // 微分ゲイン / Derivative gain

    float prev_error;   // 前回の偏差 / Previous error
    float integral;     // 積分値 / Integral accumulator
    unsigned long prev_time;  // 前回の計算時刻 / Previous calculation time

    float integral_limit;  // 積分値上限 / Integral windup limit
    float output_min;      // 出力下限 / Output minimum
    float output_max;      // 出力上限 / Output maximum

public:
    /**
     * コンストラクタ
     * @param kp 比例ゲイン / Proportional gain
     * @param ki 積分ゲイン / Integral gain
     * @param kd 微分ゲイン / Derivative gain
     * @param i_limit 積分値上限 / Integral windup limit
     * @param out_min 出力下限 / Output minimum
     * @param out_max 出力上限 / Output maximum
     */
    PIDController(float kp, float ki, float kd,
                  float i_limit, float out_min, float out_max);

    /**
     * PID計算を実行
     * Execute PID calculation
     * @param error 現在の偏差 / Current error
     * @return 制御出力 / Control output
     */
    float calculate(float error);

    /**
     * 内部状態をリセット
     * Reset internal state
     */
    void reset();

    /**
     * ゲインを動的に変更
     * Dynamically update gains
     */
    void setGains(float kp, float ki, float kd);
};

#endif // PID_CONTROLLER_H
