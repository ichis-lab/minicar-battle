# PID制御実装計画 / PID Control Implementation Plan

## 概要 / Overview

このドキュメントは、自動運転ミニカーの壁追従制御システムにPID制御を導入するための実装計画です。

This document outlines the implementation plan for introducing PID control to the autonomous minicar's wall-following control system.

---

## 現状分析 / Current State Analysis

### 現在のアーキテクチャ / Current Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Arduino Nano R4                          │
├─────────────────────────────────────────────────────────────┤
│  SensorReader → WallDetector → SteeringController → Actuator│
│      ↑              ↑                ↑               ↓      │
│  5x VL53L0X     幾何学計算      P制御のみ        Servo/ESC  │
│  (10Hz)        Geometry       P-only             PWM        │
└─────────────────────────────────────────────────────────────┘
```

### 現在の問題点 / Current Issues

| 問題 / Issue | 原因 / Cause | 影響 / Impact |
|--------------|--------------|---------------|
| オーバーシュート / Overshoot | I項・D項がない / No I or D terms | 中央復帰時に振動 / Oscillation when returning to center |
| 定常偏差 / Steady-state error | 積分項なし / No integral term | 目標距離に収束しない / Cannot converge to target distance |
| 急変化への反応遅れ / Slow response to rapid changes | 微分項なし / No derivative term | カーブ入口で追従遅れ / Lag at curve entry |
| 低速制御ループ / Slow control loop | 10Hz (100ms) | 高速走行で追従不可 / Cannot track at high speed |
| 固定速度 / Fixed speed | 動的速度制御なし / No dynamic speed control | コーナーでスリップ / Slip at corners |

### 現在のパラメータ / Current Parameters

```cpp
// Config.h - 現在の設定 / Current settings
const unsigned long MEASUREMENT_INTERVAL = 100;  // 100ms = 10Hz
const float CENTERING_GAIN = 0.05;               // P制御のみ / P-only
const float LEFT_AVOID_GAIN = 0.1;               // 回避補正 / Avoidance correction
const float MAX_STEERING_ANGLE = 30.0;           // 最大操舵角 / Max steering angle
const float BASE_SPEED_PULSE = 1.45;             // 固定速度 / Fixed speed (ms)
```

---

## 新アーキテクチャ設計 / New Architecture Design

### 目標アーキテクチャ / Target Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Arduino Nano R4                              │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌──────────────────────────┐ │
│  │SensorReader │───▶│WallDetector │───▶│    PIDController         │ │
│  │(25-50Hz)    │    │             │    │  ├─ SteeringPID          │ │
│  └─────────────┘    └─────────────┘    │  └─ SpeedPID (optional)  │ │
│        │                               └────────────┬─────────────┘ │
│        │                                            │               │
│        │            ┌─────────────┐                 │               │
│        └───────────▶│ControlMode  │─────────────────┤               │
│                     │ Manager     │                 │               │
│                     └─────────────┘                 ▼               │
│                                              ┌─────────────┐        │
│                                              │  Actuator   │        │
│                                              │ Servo + ESC │        │
│                                              └─────────────┘        │
└──────────────────────────────────────────────────────────────────────┘
```

### 新規クラス設計 / New Class Design

#### 1. PIDController クラス / PIDController Class

```cpp
// PIDController.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

struct PIDGains {
    float Kp;           // 比例ゲイン / Proportional gain
    float Ki;           // 積分ゲイン / Integral gain
    float Kd;           // 微分ゲイン / Derivative gain
};

struct PIDState {
    float prev_error;   // 前回の誤差 / Previous error
    float integral;     // 積分値 / Integral accumulator
    unsigned long last_time;  // 前回の計算時刻 / Last calculation time
};

struct PIDConfig {
    float output_min;   // 出力下限 / Output minimum
    float output_max;   // 出力上限 / Output maximum
    float integral_min; // 積分下限（アンチワインドアップ）/ Integral min (anti-windup)
    float integral_max; // 積分上限（アンチワインドアップ）/ Integral max (anti-windup)
    float deadband;     // 不感帯 / Deadband threshold
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
```

#### 2. PIDController 実装 / PIDController Implementation

```cpp
// PIDController.cpp
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
    
    // D項（測定値微分を使用して目標値急変時のキックを防止）
    // Derivative term (using measurement derivative to prevent setpoint kick)
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

float PIDController::getProportional() const { return gains.Kp * state.prev_error; }
float PIDController::getIntegral() const { return gains.Ki * state.integral; }
float PIDController::getDerivative() const { return 0.0; }  // 簡略化 / Simplified
float PIDController::getError() const { return state.prev_error; }
```

---

## センサー高速化 / Sensor Speed Optimization

### Timing Budget の設定 / Timing Budget Configuration

VL53L0Xのtiming budgetを調整して制御周波数を向上させます。

Adjust VL53L0X timing budget to improve control frequency.

#### 変更箇所 / Changes Required

```cpp
// SensorReader.cpp - begin() メソッドに追加 / Add to begin() method

bool SensorReader::begin() {
    Wire.begin();
    
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        selectChannel(SENSOR_CHANNELS[i]);
        delay(10);
        
        if (!sensors[i].begin()) {
            return false;
        }
        
        // ★追加: High Speed モードに設定 / ADD: Set High Speed mode
        // 20ms timing budget = 理論上50Hz / 20ms = theoretical 50Hz per sensor
        sensors[i].setMeasurementTimingBudget(20000);
        
        // ★追加: 連続測定モード開始 / ADD: Start continuous measurement mode
        sensors[i].startRangeContinuous();
    }
    
    return true;
}

// readAll() メソッドも変更 / Also modify readAll() method
void SensorReader::readAll() {
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        selectChannel(SENSOR_CHANNELS[i]);
        
        // 連続モードでの読み取り / Read in continuous mode
        if (sensors[i].isRangeComplete()) {
            sensorData[i].distance = sensors[i].readRange();
            sensorData[i].valid = (sensors[i].readRangeStatus() == 0);
        }
    }
}
```

### Config.h の更新 / Config.h Updates

```cpp
// Config.h - 新しいパラメータ / New parameters

// ============================================================================
// タイミング設定（更新）/ Timing Settings (Updated)
// ============================================================================
const unsigned long MEASUREMENT_INTERVAL = 40;  // 40ms = 25Hz (was 100ms = 10Hz)
const uint32_t SENSOR_TIMING_BUDGET = 20000;    // 20ms per sensor

// ============================================================================
// PID パラメータ / PID Parameters
// ============================================================================
// ステアリングPID / Steering PID
const float STEERING_KP = 0.08;    // 比例ゲイン / Proportional gain
const float STEERING_KI = 0.005;   // 積分ゲイン / Integral gain
const float STEERING_KD = 0.02;    // 微分ゲイン / Derivative gain
const float STEERING_INTEGRAL_MAX = 50.0;  // 積分上限 / Integral max

// 目標壁距離（両壁検出時）/ Target wall distance (when both walls detected)
const float TARGET_CENTER_OFFSET = 0.0;    // 中央からのオフセット / Offset from center (mm)

// 左壁追従時の目標距離 / Target distance for left wall following
const float TARGET_LEFT_DISTANCE = 400.0;  // 400mm = 40cm

// ============================================================================
// 速度制御パラメータ / Speed Control Parameters
// ============================================================================
const float SPEED_KP = 0.001;      // 速度PID比例ゲイン / Speed PID proportional gain
const float MIN_SPEED_PULSE = 1.48;   // 最小速度 / Minimum speed (ms)
const float MAX_SPEED_PULSE = 1.40;   // 最大速度 / Maximum speed (ms)
const float CORNER_SPEED_PULSE = 1.47; // コーナー速度 / Corner speed (ms)
```

---

## 新しい SteeringController / New SteeringController

### SteeringController.h（更新版）/ SteeringController.h (Updated)

```cpp
// SteeringController.h
#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "WallDetector.h"
#include "PIDController.h"

// 制御モード / Control modes
enum ControlMode {
    MODE_BOTH_WALLS,    // 両壁検出 → 中央走行 / Both walls → center driving
    MODE_LEFT_WALL,     // 左壁のみ → 左壁追従 / Left wall only → follow left
    MODE_RIGHT_WALL,    // 右壁のみ → 右壁追従 / Right wall only → follow right
    MODE_NO_WALLS       // 壁なし → 直進 / No walls → straight
};

class SteeringController {
private:
    PIDController centeringPID;     // 中央走行用PID / Centering PID
    PIDController wallFollowPID;    // 壁追従用PID / Wall following PID
    ControlMode currentMode;
    ControlMode previousMode;

public:
    SteeringController();
    
    // 初期化 / Initialize
    void begin();
    
    // ステアリング角度を計算 / Calculate steering angle
    float calculate(const WallDetection& walls, const SensorData* sensorData);
    
    // 現在のモードを取得 / Get current mode
    ControlMode getMode() const { return currentMode; }
    
    // PIDをリセット / Reset PID
    void reset();
    
    // デバッグ情報 / Debug info
    void printDebugInfo() const;
};

#endif // STEERING_CONTROLLER_H
```

### SteeringController.cpp（更新版）/ SteeringController.cpp (Updated)

```cpp
// SteeringController.cpp
#include "SteeringController.h"
#include "Logger.h"

SteeringController::SteeringController() {
    currentMode = MODE_NO_WALLS;
    previousMode = MODE_NO_WALLS;
}

void SteeringController::begin() {
    // 中央走行PID初期化 / Initialize centering PID
    centeringPID.begin(STEERING_KP, STEERING_KI, STEERING_KD);
    centeringPID.setOutputLimits(-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    centeringPID.setIntegralLimits(-STEERING_INTEGRAL_MAX, STEERING_INTEGRAL_MAX);
    centeringPID.setDeadband(10.0);  // 10mm不感帯 / 10mm deadband
    
    // 壁追従PID初期化 / Initialize wall following PID
    wallFollowPID.begin(STEERING_KP * 1.2, STEERING_KI, STEERING_KD * 1.5);
    wallFollowPID.setOutputLimits(-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    wallFollowPID.setIntegralLimits(-STEERING_INTEGRAL_MAX, STEERING_INTEGRAL_MAX);
    wallFollowPID.setDeadband(5.0);
}

float SteeringController::calculate(const WallDetection& walls, const SensorData* sensorData) {
    float steering_angle = 0.0;
    
    // モード判定 / Mode determination
    previousMode = currentMode;
    
    if (walls.left_valid && walls.right_valid) {
        currentMode = MODE_BOTH_WALLS;
    } else if (walls.left_valid) {
        currentMode = MODE_LEFT_WALL;
    } else if (walls.right_valid) {
        currentMode = MODE_RIGHT_WALL;
    } else {
        currentMode = MODE_NO_WALLS;
    }
    
    // モード変更時にPIDをリセット / Reset PID on mode change
    if (currentMode != previousMode) {
        centeringPID.reset();
        wallFollowPID.reset();
    }
    
    // モードに応じた制御 / Mode-specific control
    switch (currentMode) {
        case MODE_BOTH_WALLS: {
            // 目標: 左右壁の中央 / Target: center between walls
            // 誤差 = 右壁距離 - 左壁距離（正なら左寄り）
            // Error = right_distance - left_distance (positive = too left)
            float error = walls.right_distance - walls.left_distance;
            float setpoint = TARGET_CENTER_OFFSET;  // 通常は0 / Usually 0
            steering_angle = centeringPID.compute(setpoint, -error);
            break;
        }
        
        case MODE_LEFT_WALL: {
            // 目標: 左壁から一定距離 / Target: fixed distance from left wall
            float measured = walls.left_distance;
            steering_angle = wallFollowPID.compute(TARGET_LEFT_DISTANCE, measured);
            
            // 壁角度による補正も加える / Also add wall angle correction
            steering_angle += walls.left_angle * 0.5;
            break;
        }
        
        case MODE_RIGHT_WALL: {
            // 目標: 右壁から一定距離 / Target: fixed distance from right wall
            float measured = walls.right_distance;
            steering_angle = -wallFollowPID.compute(TARGET_LEFT_DISTANCE, measured);
            
            // 壁角度による補正 / Wall angle correction
            steering_angle -= walls.right_angle * 0.5;
            break;
        }
        
        case MODE_NO_WALLS:
        default:
            // 直進維持 / Maintain straight
            steering_angle = 0.0;
            break;
    }
    
    // =========================================================================
    // 安全制約: 左側センサーが近すぎる場合の緊急回避
    // Safety constraint: Emergency avoidance if left sensors too close
    // =========================================================================
    if (sensorData != nullptr) {
        uint16_t sensor0_dist = sensorData[0].valid ? sensorData[0].distance : 9999;
        uint16_t sensor1_dist = sensorData[1].valid ? sensorData[1].distance : 9999;
        uint16_t min_left_dist = min(sensor0_dist, sensor1_dist);
        
        if (min_left_dist < MIN_LEFT_DISTANCE) {
            float shortage = MIN_LEFT_DISTANCE - min_left_dist;
            steering_angle += shortage * LEFT_AVOID_GAIN;
        }
    }
    
    // 最大操舵角でクランプ / Clamp to max steering angle
    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    
    return steering_angle;
}

void SteeringController::reset() {
    centeringPID.reset();
    wallFollowPID.reset();
    currentMode = MODE_NO_WALLS;
    previousMode = MODE_NO_WALLS;
}

void SteeringController::printDebugInfo() const {
    Logger::print(" Mode:");
    switch (currentMode) {
        case MODE_BOTH_WALLS: Logger::print("BOTH"); break;
        case MODE_LEFT_WALL:  Logger::print("LEFT"); break;
        case MODE_RIGHT_WALL: Logger::print("RIGHT"); break;
        case MODE_NO_WALLS:   Logger::print("NONE"); break;
    }
}
```

---

## 実装手順 / Implementation Steps

### Phase 1: PIDControllerクラスの追加 / Add PIDController Class

1. `PIDController.h` を作成 / Create `PIDController.h`
2. `PIDController.cpp` を作成 / Create `PIDController.cpp`
3. 単体テストを実施 / Run unit tests

```bash
# ファイル構成 / File structure
five_sensor_reader/
├── Config.h              # 更新 / Update
├── PIDController.h       # 新規 / New
├── PIDController.cpp     # 新規 / New
├── SteeringController.h  # 更新 / Update
├── SteeringController.cpp # 更新 / Update
├── SensorReader.h
├── SensorReader.cpp      # 更新 / Update
├── WallDetector.h
├── WallDetector.cpp
├── Actuator.h
├── Actuator.cpp
├── Logger.h
└── five_sensor_reader.ino # 更新 / Update
```

### Phase 2: センサー高速化 / Sensor Speed Optimization

1. `SensorReader.cpp` の `begin()` に timing budget 設定を追加
   Add timing budget setting to `SensorReader.cpp` `begin()`
2. `Config.h` の `MEASUREMENT_INTERVAL` を 100ms → 40ms に変更
   Change `MEASUREMENT_INTERVAL` from 100ms to 40ms in `Config.h`
3. 動作確認（シリアル出力で制御周波数を確認）
   Verify operation (check control frequency via serial output)

### Phase 3: SteeringController の更新 / Update SteeringController

1. PIDController を使用するように SteeringController を更新
   Update SteeringController to use PIDController
2. モード切替ロジックの実装
   Implement mode switching logic
3. デバッグ出力の追加
   Add debug output

### Phase 4: パラメータチューニング / Parameter Tuning

1. P項のみでテスト（Ki=0, Kd=0）
   Test with P term only (Ki=0, Kd=0)
2. D項を追加してオーバーシュート抑制
   Add D term to suppress overshoot
3. I項を追加して定常偏差を解消
   Add I term to eliminate steady-state error
4. 各モードでのパラメータ調整
   Adjust parameters for each mode

---

## チューニングガイド / Tuning Guide

### 推奨チューニング手順 / Recommended Tuning Procedure

```
1. Kp のみで開始 / Start with Kp only
   - 振動が始まるまで Kp を増加 / Increase Kp until oscillation starts
   - その値の 50-60% を使用 / Use 50-60% of that value

2. Kd を追加 / Add Kd
   - オーバーシュートが減少するまで Kd を増加
   - Increase Kd until overshoot decreases
   - 応答が遅くなりすぎない程度に / Don't make response too slow

3. Ki を追加 / Add Ki
   - 定常偏差がなくなるまで Ki を増加
   - Increase Ki until steady-state error disappears
   - ハンチングに注意 / Watch for hunting
```

### 初期パラメータ推奨値 / Recommended Initial Parameters

| パラメータ / Parameter | 初期値 / Initial | 調整範囲 / Range |
|------------------------|------------------|------------------|
| STEERING_KP | 0.08 | 0.03 - 0.15 |
| STEERING_KI | 0.005 | 0.001 - 0.02 |
| STEERING_KD | 0.02 | 0.005 - 0.05 |
| MEASUREMENT_INTERVAL | 40ms | 20ms - 100ms |

---

## テスト計画 / Test Plan

### 単体テスト / Unit Tests

1. **PIDController テスト**
   - ステップ応答テスト / Step response test
   - アンチワインドアップ動作確認 / Anti-windup verification
   - 出力制限動作確認 / Output limiting verification

2. **センサー高速化テスト**
   - 実際の制御周波数測定 / Actual control frequency measurement
   - センサー読み取りエラー率 / Sensor read error rate

### 統合テスト / Integration Tests

1. **直線走行テスト**
   - 中央維持性能 / Center maintaining performance
   - オーバーシュート量 / Overshoot amount

2. **カーブ走行テスト**
   - 追従性能 / Tracking performance
   - 遅延量 / Delay amount

3. **速度変化テスト**
   - 各速度での安定性 / Stability at each speed

---

## 注意事項 / Notes

### アンチワインドアップ / Anti-Windup

積分値の蓄積を制限して、急激な方向転換時の暴走を防止します。

Limit integral accumulation to prevent runaway during rapid direction changes.

```cpp
// 積分値の制限 / Limit integral value
state.integral = constrain(state.integral, config.integral_min, config.integral_max);
```

### モード切替時のリセット / Reset on Mode Change

制御モードが変わった時は、PID状態をリセットして不連続な出力を防止します。

Reset PID state when control mode changes to prevent discontinuous output.

```cpp
if (currentMode != previousMode) {
    centeringPID.reset();
    wallFollowPID.reset();
}
```

### デバッグモードの活用 / Using Debug Mode

実機テスト前に `DEBUG_MODE = true` で動作確認することを推奨します。

Recommend verifying operation with `DEBUG_MODE = true` before real machine testing.

---

## 参考資料 / References

- VL53L0X データシート / Datasheet
- VL53L1X データシート / Datasheet  
- プロジェクト「PID制御について」ドキュメント / Project "About PID Control" document
- Arduino PID Library 設計パターン / Arduino PID Library design patterns

---

## 変更履歴 / Change Log

| 日付 / Date | 変更内容 / Changes |
|-------------|-------------------|
| 2024-XX-XX | 初版作成 / Initial creation |
| 2024-12-22 | 実装完了 + 設計変更（下記参照） |

---

## 実装状況 / Implementation Status (2024-12-22)

### 実装完了項目 ✅

| 項目 | 状態 | 備考 |
|------|------|------|
| PIDControllerクラス | ✅ 完了 | アンチワインドアップ、不感帯、微分フィルタ対応 |
| 壁距離計算の修正 | ✅ 完了 | 2点平均 → 直線と原点の垂直距離 |
| 微分フィルタ | ✅ 完了 | ローパスフィルタ（α=0.3） |
| 両壁モード | ✅ 完了 | 距離PIDで中央維持 |
| 片壁モード | ✅ 完了 | 角度PIDで壁と平行維持 |

### 設計変更点

#### 片壁モードの制御方針変更

**当初計画**: 壁から400mm離れた位置を維持（距離PID）

**実装**: 壁と平行に走行（角度PID）+ 安全距離制約（150mm）

**変更理由**:
- 片壁のみの状況で無理に距離を維持するのはリスクが高い
- 壁と平行に走り、近すぎる場合のみ離れる方が安全
- 直進できる場所は素直に直進（将来の加速制御に対応）

#### パラメータの整理

**削除（8個）**: 未使用パラメータを削除
- `SENSOR_ERROR_VALUE`, `SENSOR_TIMING_BUDGET`
- `SPEED_KP`, `MIN_SPEED_PULSE`, `MAX_SPEED_PULSE`, `CORNER_SPEED_PULSE`
- `TARGET_LEFT_DISTANCE`, `MIN_LEFT_DISTANCE`

**追加（6個）**: 新制御用パラメータ
- `ANGLE_KP=0.5`, `ANGLE_KI=0.0`, `ANGLE_KD=0.1`（角度PID）
- `MIN_SAFE_DISTANCE=150`（最低安全距離、左右共通）
- `DISTANCE_AVOID_GAIN=0.15`（距離制約ゲイン）
- `DERIVATIVE_FILTER_ALPHA=0.3`（微分フィルタ係数）

### 最終的なアーキテクチャ

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Arduino Nano R4                              │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌──────────────────────────┐ │
│  │SensorReader │───▶│WallDetector │───▶│  SteeringController      │ │
│  │(25Hz)       │    │(垂直距離計算)│    │  ├─ centeringPID(距離)   │ │
│  └─────────────┘    └─────────────┘    │  └─ anglePID(角度)       │ │
│                                        └────────────┬─────────────┘ │
│                                                     │               │
│                                                     ▼               │
│                                              ┌─────────────┐        │
│                                              │  Actuator   │        │
│                                              │ Servo + ESC │        │
│                                              └─────────────┘        │
└──────────────────────────────────────────────────────────────────────┘
```

### 制御モード詳細

| モード | 目標 | 制御方法 |
|--------|------|----------|
| BOTH_WALLS | 中央走行 | centeringPID: 距離差→0 |
| LEFT_WALL | 壁と平行 | anglePID: 角度→0° + 安全距離制約 |
| RIGHT_WALL | 壁と平行 | anglePID: 角度→0° + 安全距離制約 |
| NO_WALLS | 直進 | ステアリング=0° |

### 未実装項目（将来課題）

| 項目 | 状態 | 備考 |
|------|------|------|
| センサー高速化（連続測定モード） | ❌ 未実装 | 現在はブロッキング読み取り |
| 速度PID制御 | ❌ 未実装 | 現在は固定速度 |
| コーナー出口の挙動 | ❌ 未検討 | 片壁→壁なし遷移時の対応 |