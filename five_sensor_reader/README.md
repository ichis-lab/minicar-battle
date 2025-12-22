# Five Sensor Reader - PID制御壁追従システム

## 概要

5つのVL53L0X ToFセンサーを使用した自動運転ミニカーの壁追従制御システム。
PID制御により、オーバーシュート抑制、定常偏差解消、急変化への追従を実現。

## アーキテクチャ

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Arduino Nano R4                              │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌──────────────────────────┐ │
│  │SensorReader │───▶│WallDetector │───▶│  SteeringController      │ │
│  │(25Hz)       │    │             │    │  ├─ centeringPID         │ │
│  └─────────────┘    └─────────────┘    │  └─ wallFollowPID        │ │
│        │                               └────────────┬─────────────┘ │
│        │                                            │               │
│        │                                            ▼               │
│        │                                     ┌─────────────┐        │
│        └────────────────────────────────────▶│  Actuator   │        │
│                                              │ Servo + ESC │        │
│                                              └─────────────┘        │
└──────────────────────────────────────────────────────────────────────┘
```

## ファイル構成

| ファイル | 役割 |
|---------|------|
| `five_sensor_reader.ino` | メインループ、全体の制御フロー |
| `Config.h` | 定数・パラメータの一元管理 |
| `SensorReader.h/.cpp` | I2Cマルチプレクサ経由でセンサー読取（高速化対応） |
| `WallDetector.h/.cpp` | センサーデータから壁を検出、距離・角度を計算 |
| `PIDController.h/.cpp` | 汎用PID制御器（アンチワインドアップ、不感帯対応） |
| `SteeringController.h/.cpp` | 制御モード判定、2つのPIDでステアリング決定 |
| `Actuator.h/.cpp` | サーボ・ESCへのPWM出力 |
| `Logger.h` | デバッグ出力（DEBUG_MODEで切替） |

## 処理フロー

### メインループ (40ms = 25Hz)

```
┌─────────────────────────────────────────────────────────────┐
│ Phase 1: センサーデータ取得                                   │
│   sensorReader.readAll()                                     │
│   → 5つのセンサーから距離データを取得（連続測定モード）        │
└─────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Phase 2: 緊急停止チェック                                     │
│   前方センサー(Index 2) < EMERGENCY_FRONT_THRESHOLD          │
│   → true: 緊急停止フラグON                                   │
└─────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Phase 3: 壁検出                                              │
│   wallDetector.detect(sensorData)                            │
│   → 左右の壁の有無、距離、角度を計算                          │
└─────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Phase 4: ステアリング角度計算（PID制御）                      │
│   steeringController.calculate(walls, sensorData)            │
│   → 制御モードに応じたPIDでステアリング角度を算出            │
└─────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Phase 5: アクチュエーター制御                                 │
│   緊急停止時: ステアリング中央 + モーター停止                   │
│   通常時:     計算したステアリング角度 + 基本速度              │
└─────────────────────────────────────────────────────────────┘
```

## 制御モード

| モード | 条件 | 制御方法 |
|--------|------|----------|
| `MODE_BOTH_WALLS` | 左右両方の壁を検出 | centeringPID で中央を維持 |
| `MODE_LEFT_WALL` | 左壁のみ検出 | wallFollowPID で左壁から一定距離を維持 |
| `MODE_RIGHT_WALL` | 右壁のみ検出 | wallFollowPID で右壁から一定距離を維持 |
| `MODE_NO_WALLS` | 壁なし | 直進 |

## PID制御の詳細

### PIDController クラス

```cpp
// 構造体ベースの設計
struct PIDGains { Kp, Ki, Kd };
struct PIDState { prev_error, integral, last_time };
struct PIDConfig { output_min/max, integral_min/max, deadband };

// 主要メソッド
void begin(Kp, Ki, Kd);           // 初期化
float compute(setpoint, measured); // PID計算
void reset();                      // 状態リセット
```

### アンチワインドアップ

積分値の蓄積を制限して、急激な方向転換時の暴走を防止:

```cpp
state.integral = constrain(state.integral, config.integral_min, config.integral_max);
```

### モード切替時のリセット

制御モードが変わった時はPID状態をリセット:

```cpp
if (currentMode != previousMode) {
    centeringPID.reset();
    wallFollowPID.reset();
}
```

## 主要パラメータ (Config.h)

### タイミング

```cpp
MEASUREMENT_INTERVAL = 40    // 40ms = 25Hz
SENSOR_TIMING_BUDGET = 20000 // 20ms per sensor
```

### PIDゲイン

```cpp
STEERING_KP = 0.08           // 比例ゲイン
STEERING_KI = 0.005          // 積分ゲイン
STEERING_KD = 0.02           // 微分ゲイン
STEERING_INTEGRAL_MAX = 50.0 // 積分上限
```

### 目標距離

```cpp
TARGET_CENTER_OFFSET = 0.0   // 中央走行時のオフセット(mm)
TARGET_LEFT_DISTANCE = 400.0 // 壁追従時の目標距離(mm)
```

### 安全パラメータ

```cpp
MIN_LEFT_DISTANCE = 300           // 左側最小距離(mm)
LEFT_AVOID_GAIN = 0.1             // 回避補正ゲイン
EMERGENCY_FRONT_THRESHOLD = 200   // 前方緊急閾値(mm)
```

## チューニングガイド

### 推奨手順

```
1. Kp のみで開始 (Ki=0, Kd=0)
   - 振動が始まるまで Kp を増加
   - その値の 50-60% を使用

2. Kd を追加
   - オーバーシュートが減少するまで Kd を増加
   - 応答が遅くなりすぎない程度に

3. Ki を追加
   - 定常偏差がなくなるまで Ki を増加
   - ハンチングに注意
```

### パラメータ調整範囲

| パラメータ | 初期値 | 調整範囲 |
|-----------|--------|----------|
| STEERING_KP | 0.08 | 0.03 - 0.15 |
| STEERING_KI | 0.005 | 0.001 - 0.02 |
| STEERING_KD | 0.02 | 0.005 - 0.05 |

## センサー配置

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

## デバッグ方法

`Config.h` で `DEBUG_MODE` を `true` に設定:

```cpp
#define DEBUG_MODE true
```

シリアル出力例:
```
S0:350 | S1:800 | S2:1200 | S3:600 | S4:300 | W:LR D:575/450 Mode:BOTH St:-2.5
```

出力フォーマット:
- `S0:350` - センサー0の距離(mm)
- `W:LR` - 壁検出状態 (L=左, R=右)
- `D:575/450` - 左/右の壁までの距離(mm)
- `Mode:BOTH` - 制御モード
- `St:-2.5` - ステアリング角度(度)

## 高速化について

### Timing Budget

VL53L0Xのtiming budgetを20msに設定し、制御周波数を10Hz→25Hzに向上:

```cpp
sensors[i].setMeasurementTimingBudget(SENSOR_TIMING_BUDGET); // 20000μs
sensors[i].startRangeContinuous();
```

### 連続測定モード

ブロッキングなしでセンサー値を取得:

```cpp
if (sensors[i].isRangeComplete()) {
    uint16_t range = sensors[i].readRange();
    // ...
}
```
