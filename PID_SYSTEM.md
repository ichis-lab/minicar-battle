# Openness-Based PID Control Implementation Plan
# 開放度ベースPID制御 実装計画

## 目的 / Purpose

左壁追従から「左右の開放度バランス」へ制御目標を転換し、
広いセクションでの不要な蛇行を防ぐPID制御を実装する。

Implement PID control that shifts from "left wall following" to 
"left-right openness balance" to prevent unnecessary oscillation 
in wide sections.

---

## 現状 / Current State

### 既存ファイル構成 / Existing File Structure

```
five_sensor_reader/
├── five_sensor_reader.ino  # メインループ
├── Config.h                # 定数・設定値
├── SensorManager.h/.cpp    # センサー管理
├── WallDetector.h/.cpp     # 壁検出（置き換え対象）
├── SteeringController.h/.cpp # ステアリング制御（置き換え対象）
├── Actuator.h/.cpp         # アクチュエーター制御
└── Logger.h/.cpp           # ログ出力
```

### センサー配置 / Sensor Layout

```
Index 0: -70° (左側方)
Index 1: -20° (左前方)
Index 2:   0° (前方)
Index 3: +20° (右前方)
Index 4: +70° (右側方)
```

---

## 実装内容 / Implementation Scope

### 新規作成ファイル / New Files to Create

```
five_sensor_reader/
├── OpennessCalculator.h    # 開放度計算クラス（宣言）
├── OpennessCalculator.cpp  # 開放度計算クラス（実装）
├── PIDController.h         # 汎用PIDコントローラ（宣言）
└── PIDController.cpp       # 汎用PIDコントローラ（実装）
```

### 修正ファイル / Files to Modify

```
├── Config.h                # 新パラメータ追加
├── SteeringController.h/.cpp # 内部ロジック置き換え
└── five_sensor_reader.ino  # 必要に応じて調整
```

---

## クラス設計 / Class Design

### 1. PIDController（汎用PIDコントローラ）

```cpp
// PIDController.h

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * 汎用PIDコントローラ
 * Generic PID Controller
 * 
 * 任意の制御対象に使用可能な汎用PID実装
 * Reusable PID implementation for any control target
 */
class PIDController {
private:
    float Kp;
    float Ki;
    float Kd;
    
    float prev_error;
    float integral;
    unsigned long prev_time;
    
    float integral_limit;
    float output_min;
    float output_max;

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
     * ゲインを動的に変更（オプション）
     * Dynamically update gains (optional)
     */
    void setGains(float kp, float ki, float kd);
};

#endif // PID_CONTROLLER_H
```

### 2. OpennessCalculator（開放度計算）

```cpp
// OpennessCalculator.h

#ifndef OPENNESS_CALCULATOR_H
#define OPENNESS_CALCULATOR_H

#include <Arduino.h>
#include "Config.h"

/**
 * 開放度計算結果
 * Openness calculation result
 */
struct OpennessData {
    float left_openness;    // 左側開放度 / Left side openness
    float right_openness;   // 右側開放度 / Right side openness
    float error;            // 偏差 (right - left) / Error
    bool valid;             // 計算が有効か / Calculation validity
};

/**
 * 開放度計算クラス
 * Openness Calculator
 * 
 * センサーデータから左右の開放度を計算
 * Calculates left/right openness from sensor data
 */
class OpennessCalculator {
private:
    float weight_far;   // 70°センサーの重み
    float weight_near;  // 20°センサーの重み
    
    /**
     * 片側の開放度を計算
     * Calculate openness for one side
     */
    float calculateSideOpenness(uint16_t dist_far, uint16_t dist_near);
    
    /**
     * センサー値の妥当性チェック
     * Validate sensor reading
     */
    bool isValidReading(uint16_t distance);

public:
    /**
     * コンストラクタ
     * @param w_far 70°センサーの重み / Weight for 70° sensors
     * @param w_near 20°センサーの重み / Weight for 20° sensors
     */
    OpennessCalculator(float w_far, float w_near);
    
    /**
     * 開放度を計算
     * Calculate openness
     * @param distances センサー距離配列[5] / Sensor distance array[5]
     * @return 開放度データ / Openness data
     */
    OpennessData calculate(const uint16_t distances[5]);
};

#endif // OPENNESS_CALCULATOR_H
```

### 3. SteeringController（修正版）

```cpp
// SteeringController.h (修正版 / Modified)

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "OpennessCalculator.h"
#include "PIDController.h"

// 前方宣言 / Forward declaration
struct SensorData;

/**
 * ステアリング制御クラス（開放度ベース）
 * Steering Controller (Openness-based)
 */
class SteeringController {
private:
    OpennessCalculator opennessCalc;
    PIDController pid;
    
    /**
     * 緊急回避が必要か判定
     * Check if emergency avoidance is needed
     */
    bool needsEmergencyAvoidance(uint16_t front_distance);
    
    /**
     * 緊急回避時のステアリング計算
     * Calculate steering for emergency avoidance
     */
    float calculateEmergencySteering(const OpennessData& openness);

public:
    /**
     * コンストラクタ
     */
    SteeringController();
    
    /**
     * ステアリング角度を計算
     * Calculate steering angle
     * @param sensorData センサーデータ配列[5]
     * @return ステアリング角度（度） / Steering angle in degrees
     */
    float calculate(const SensorData* sensorData);
    
    /**
     * 状態リセット
     * Reset state
     */
    void reset();
};

#endif // STEERING_CONTROLLER_H
```

---

## Config.h への追加パラメータ / New Parameters for Config.h

```cpp
// ============================================================================
// 開放度ベースPID制御パラメータ
// Openness-based PID Control Parameters
// ============================================================================

// 開放度計算の重み / Openness calculation weights
const float OPENNESS_WEIGHT_FAR = 0.5;   // 70°センサー重み
const float OPENNESS_WEIGHT_NEAR = 0.5;  // 20°センサー重み

// PIDゲイン / PID gains
// 注: 初期値は実験で調整が必要
// Note: Initial values need tuning through experiments
const float OPENNESS_PID_KP = 0.02;      // 比例ゲイン
const float OPENNESS_PID_KI = 0.0;       // 積分ゲイン（初期は0）
const float OPENNESS_PID_KD = 0.0;       // 微分ゲイン（初期は0）

// PID制限値 / PID limits
const float OPENNESS_PID_INTEGRAL_LIMIT = 500.0;  // 積分上限

// 緊急回避パラメータ / Emergency avoidance parameters
const uint16_t EMERGENCY_FRONT_THRESHOLD = 200;   // 前方緊急閾値(mm)
```

---

## 実装手順 / Implementation Steps

### Step 1: PIDController クラス作成

```
1. PIDController.h を作成
2. PIDController.cpp を作成
3. 単体テスト（シリアル出力で動作確認）
```

### Step 2: OpennessCalculator クラス作成

```
1. OpennessCalculator.h を作成
2. OpennessCalculator.cpp を作成
3. 単体テスト（センサー値を入れて開放度を出力）
```

### Step 3: Config.h 更新

```
1. 新パラメータを追加
2. 既存パラメータは残す（互換性のため）
```

### Step 4: SteeringController 修正

```
1. 内部で OpennessCalculator と PIDController を使用するよう変更
2. calculate() メソッドのロジック置き換え
3. 緊急回避ロジックを追加
```

### Step 5: 統合テスト

```
1. DEBUG_MODE=true でセンサー値と計算結果を確認
2. 開放度、error、ステアリング角度をシリアル出力
3. 期待通りの値か確認
```

---

## 設計上の制約 / Design Constraints

### MUST（必須）

- [ ] オブジェクト指向で実装（クラスベース）
- [ ] 1クラス = 1ファイルペア（.h / .cpp）
- [ ] Config.h で全パラメータを一元管理
- [ ] 既存の SensorData 構造体を流用
- [ ] DEBUG_MODE 対応（シリアル出力切り替え）

### SHOULD（推奨）

- [ ] 既存コードとの互換性維持（段階的移行可能）
- [ ] ログ出力は Logger クラス経由
- [ ] マジックナンバー禁止（定数化）

### SHOULD NOT（避けるべき）

- [ ] グローバル変数の使用
- [ ] 複雑な継承関係
- [ ] 過度な抽象化

---

## テスト観点 / Test Considerations

### 単体テスト

```cpp
// PIDController テスト
PIDController pid(0.02, 0.0, 0.0, 500.0, -30.0, 30.0);
float output = pid.calculate(100.0);  // error=100 → output≈2.0

// OpennessCalculator テスト
OpennessCalculator calc(0.5, 0.5);
uint16_t distances[5] = {300, 800, 1200, 600, 250};
OpennessData data = calc.calculate(distances);
// left_openness = 0.5*300 + 0.5*800 = 550
// right_openness = 0.5*250 + 0.5*600 = 425
// error = 425 - 550 = -125 → 左が開けてる
```

### 統合テスト

```
シナリオ1: 左右均等
  距離: [400, 800, 1000, 800, 400]
  期待: error ≈ 0, steering ≈ 0°（直進）

シナリオ2: 左が開けてる
  距離: [600, 1000, 1000, 500, 300]
  期待: error < 0, steering < 0°（左へ）

シナリオ3: 前方障害物
  距離: [400, 600, 150, 800, 500]
  期待: 緊急回避発動、右へ最大舵角
```

---

## 注意事項 / Notes

### チューニングについて

```
Kp = 0.02 は初期値。実機テストで以下を観察して調整：
- 反応が鈍い → Kp を上げる
- 振動する → Kp を下げる
- 定常偏差が残る → Ki を少し入れる
- 急変に弱い → Kd を少し入れる
```

### 既存コードとの関係

```
WallDetector は当面残す（比較検証用）
SteeringController の内部実装のみ変更
外部インターフェース（calculate メソッド）は維持
```

---

## 参考：期待される挙動 / Expected Behavior

### 広いS字セクション

```
    ┌───┐     ┌───┐
    │   └─────┘   │
    │             │
    │     🚗→    │  
    │             │

センサー: 左右とも遠い
→ left_openness ≈ right_openness
→ error ≈ 0
→ steering ≈ 0°
→ 直進！（蛇行しない）
```

### コーナー進入

```
壁壁壁壁壁壁壁
              
   🚗→   壁壁
         壁壁

センサー: 右前方(+20°)が近い
→ right_openness 減少
→ error < 0
→ steering < 0°
→ 左へステアリング
```