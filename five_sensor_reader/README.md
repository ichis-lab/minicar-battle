# Five Sensor Reader - 開放度ベースPID制御システム

## 概要

5つのVL53L0X ToFセンサーを使用した自動運転ミニカーの制御システム。
左右の「開放度」（空間の広さ）のバランスを制御目標とするPID制御を実装。

## アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                    five_sensor_reader.ino                    │
│                       (メインループ)                          │
└─────────────────────────────────────────────────────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌──────────────┐    ┌──────────────────┐    ┌──────────────┐
│ SensorReader │    │SteeringController│    │   Actuator   │
│  センサー読取  │    │   ステアリング制御  │    │ PWM出力制御   │
└──────────────┘    └──────────────────┘    └──────────────┘
                            │
              ┌─────────────┴─────────────┐
              ▼                           ▼
    ┌──────────────────┐        ┌──────────────────┐
    │OpennessCalculator│        │  PIDController   │
    │    開放度計算      │        │    PID制御       │
    └──────────────────┘        └──────────────────┘
```

## 処理フロー

### 1. 初期化フェーズ (setup)

```cpp
void setup() {
  Logger::begin(9600);           // シリアル通信初期化
  sensorReader.begin();          // I2Cマルチプレクサ経由でセンサー初期化
  actuator.begin();              // サーボ・ESC初期化
}
```

**SensorReader::begin()の内部処理:**
1. TCA9548A I2Cマルチプレクサに接続
2. チャンネル0-4を順次選択
3. 各VL53L0Xセンサーを初期化

### 2. メインループ (loop)

100ms間隔（10Hz）で以下を実行:

```
┌─────────────────────────────────────────────────────────────┐
│ Phase 1: センサーデータ取得                                   │
│   sensorReader.readAll()                                     │
│   → 5つのセンサーから距離データを取得                          │
└─────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Phase 2: 緊急停止チェック                                     │
│   前方センサー(Index 2) < EMERGENCY_FRONT_THRESHOLD          │
│   → true: 緊急停止フラグON                                   │
└─────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Phase 3: ステアリング角度計算                                 │
│   steeringController.calculate(sensorData)                   │
│   → 開放度ベースPIDでステアリング角度を算出                    │
└─────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Phase 4: アクチュエーター制御                                 │
│   緊急停止時: ステアリング中央 + モーター停止                   │
│   通常時:     計算したステアリング角度 + 基本速度              │
└─────────────────────────────────────────────────────────────┘
```

## 開放度計算の詳細

### センサー配置

```
        前方 (Index 2: 0°)
              │
       -20°   │   +20°
     (Idx 1)  │  (Idx 3)
          \   │   /
           \  │  /
-70°        \ │ /        +70°
(Idx 0)      \│/      (Idx 4)
           [車体]
```

### OpennessCalculator::calculate()

```cpp
// 左側開放度 = weight_far × dist[0] + weight_near × dist[1]
// 右側開放度 = weight_far × dist[4] + weight_near × dist[3]
// 偏差(error) = 右側開放度 - 左側開放度
```

**計算例:**
```
センサー値: [300, 800, 1200, 600, 250] (mm)
重み: far=0.5, near=0.5

左側開放度 = 0.5 × 300 + 0.5 × 800 = 550
右側開放度 = 0.5 × 250 + 0.5 × 600 = 425
error = 425 - 550 = -125

→ 左が開けている → 左へステアリング
```

## PID制御の詳細

### PIDController::calculate()

```cpp
float calculate(float error) {
    // dt計算（前回からの経過時間）
    float dt = (current_time - prev_time) / 1000.0;

    // P項: 現在の偏差に比例
    float P = Kp * error;

    // I項: 偏差の積分（アンチワインドアップ付き）
    integral += error * dt;
    integral = constrain(integral, -integral_limit, integral_limit);
    float I = Ki * integral;

    // D項: 偏差の変化率
    float derivative = (error - prev_error) / dt;
    float D = Kd * derivative;

    // 出力 = P + I + D（出力制限あり）
    return constrain(P + I + D, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
}
```

### 初期パラメータ

```cpp
Kp = 0.02   // 比例ゲイン（まず調整）
Ki = 0.0    // 積分ゲイン（定常偏差があれば追加）
Kd = 0.0    // 微分ゲイン（振動抑制に使用）
```

## 緊急回避ロジック

### SteeringController内の処理

```cpp
if (front_distance < EMERGENCY_FRONT_THRESHOLD) {
    // 緊急回避モード
    if (left_openness > right_openness) {
        return -MAX_STEERING_ANGLE;  // 左へ最大舵角
    } else {
        return +MAX_STEERING_ANGLE;  // 右へ最大舵角
    }
}
```

## ファイル構成

| ファイル | 役割 |
|---------|------|
| `five_sensor_reader.ino` | メインループ、全体の制御フロー |
| `Config.h` | 定数・パラメータの一元管理 |
| `SensorReader.h/.cpp` | I2Cマルチプレクサ経由でセンサー読取 |
| `OpennessCalculator.h/.cpp` | 左右の開放度計算 |
| `PIDController.h/.cpp` | 汎用PID制御器 |
| `SteeringController.h/.cpp` | 開放度+PIDでステアリング角度決定 |
| `Actuator.h/.cpp` | サーボ・ESCへのPWM出力 |
| `Logger.h` | デバッグ出力（DEBUG_MODEで切替） |

## 主要な設定パラメータ (Config.h)

```cpp
// センサー
const uint16_t MIN_VALID_DISTANCE = 50;     // 最小有効距離(mm)
const uint16_t RELIABLE_RANGE = 1200;       // 信頼範囲(mm)

// 開放度計算
const float OPENNESS_WEIGHT_FAR = 0.5;      // 70°センサー重み
const float OPENNESS_WEIGHT_NEAR = 0.5;     // 20°センサー重み

// PID
const float OPENNESS_PID_KP = 0.02;         // 比例ゲイン
const float OPENNESS_PID_KI = 0.0;          // 積分ゲイン
const float OPENNESS_PID_KD = 0.0;          // 微分ゲイン

// 緊急回避
const uint16_t EMERGENCY_FRONT_THRESHOLD = 200;  // 前方閾値(mm)

// 速度
const float BASE_SPEED_PULSE = 1.45;        // 基本速度(ms)
```

## チューニングガイド

### Kp（比例ゲイン）の調整

1. Ki=0, Kd=0 の状態で開始
2. Kp を小さい値（0.01）から徐々に増加
3. **反応が鈍い** → Kp を上げる
4. **振動する** → Kp を下げる

### Ki（積分ゲイン）の調整

- 定常偏差（常に片側に寄る）がある場合のみ追加
- 小さい値（0.001程度）から開始

### Kd（微分ゲイン）の調整

- 急な変化に対する反応が遅い場合に追加
- ノイズに敏感なので注意

## デバッグ方法

`Config.h` で `DEBUG_MODE` を `true` に設定:

```cpp
#define DEBUG_MODE true
```

シリアル出力例:
```
Ch0: 300mm  |  Ch1: 800mm  |  Ch2: 1200mm  |  Ch3: 600mm  |  Ch4: 250mm | Open L:550 R:425 Err:-125 | Steer:-2.5deg
```

## 期待される挙動

### 広いストレート
```
左右均等 → error ≈ 0 → steering ≈ 0° → 直進
```

### 左コーナー進入
```
右が狭い → error < 0 → steering < 0° → 左へ
```

### 右コーナー進入
```
左が狭い → error > 0 → steering > 0° → 右へ
```

### 前方障害物
```
前方 < 200mm → 緊急回避 → 開けた方へ最大舵角
```
