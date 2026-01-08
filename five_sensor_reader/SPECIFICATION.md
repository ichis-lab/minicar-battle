# Five Sensor Reader 仕様設計書

## 1. システム概要

### 1.1 目的
5つのVL53L1X ToFセンサーを使用し、Follow the Gapアルゴリズムにより自動走行を行うArduino制御システム。

### 1.2 機能要件
| ID | 機能 | 説明 |
|----|------|------|
| F01 | センサー読み取り | 5つのVL53L1Xセンサーから距離データを取得 |
| F02 | ギャップ検出 | Follow the Gapアルゴリズムで通過可能な空間を検出 |
| F03 | ステアリング制御 | PD制御によりサーボモーターを制御 |
| F04 | 速度制御 | ステアリング角度に連動した速度調整 |
| F05 | 緊急停止 | 前方障害物検出時の自動停止 |
| F06 | リモート停止 | Bluetooth経由の停止コマンド受信 |
| F07 | デバッグモード | PWM無効・シリアル出力有効のテストモード |

### 1.3 ハードウェア構成

```
┌─────────────────────────────────────────────────────────────┐
│                    Arduino Nano R4                          │
│                                                             │
│  I2C (SDA/SCL)          Serial1 (TX1/RX1)    PWM (D9/D10)  │
└───────┬───────────────────────┬───────────────────┬────────┘
        │                       │                   │
        ▼                       ▼                   ▼
┌───────────────┐       ┌───────────────┐   ┌─────────────┐
│   TCA9548A    │       │    HC-06      │   │  サーボ(D9) │
│ I2Cマルチプレクサ│       │  Bluetooth    │   │  ESC(D10)   │
└───────┬───────┘       └───────────────┘   └─────────────┘
        │
   ┌────┴────┬────┬────┬────┐
   ▼         ▼    ▼    ▼    ▼
┌─────┐  ┌─────┐ ... (CH0-CH4)
│VL53 │  │VL53 │
│L1X  │  │L1X  │
└─────┘  └─────┘
```

### 1.4 センサー配置

```
              正面 (Sensor 2: 0°)
                    │
           -20°     │     +20°
             \      │      /
              \     │     /
    -70°       \    │    /       +70°
  (Sensor 0)   (S1) │ (S3)   (Sensor 4)
     左              │              右
```

| インデックス | チャンネル | 取付角度 | 用途 |
|-------------|-----------|---------|------|
| 0 | CH0 | -70° | 左側方監視 |
| 1 | CH1 | -20° | 左前方監視 |
| 2 | CH2 | 0° | 正面監視 |
| 3 | CH3 | +20° | 右前方監視 |
| 4 | CH4 | +70° | 右側方監視 |

---

## 2. クラス設計

### 2.1 クラス図

```
┌──────────────────────────────────────────────────────────────────┐
│                         <<main>>                                 │
│                    five_sensor_reader.ino                        │
│──────────────────────────────────────────────────────────────────│
│ - sensorReader: SensorReader                                     │
│ - gapFinder: GapFinder                                          │
│ - steeringController: SteeringController                        │
│ - actuator: Actuator                                            │
│──────────────────────────────────────────────────────────────────│
│ + setup(): void                                                  │
│ + loop(): void                                                   │
└──────────────────────────────────────────────────────────────────┘
         │使用
         ▼
┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐
│    SensorReader    │  │     GapFinder      │  │ SteeringController │
├────────────────────┤  ├────────────────────┤  ├────────────────────┤
│ - _sensors[]       │  │ - _gaps[]          │  │ - _lastTargetAngle │
│ - _sensorData[]    │  │ - _numGaps         │  ├────────────────────┤
├────────────────────┤  │ - _inflatedDist[]  │  │ + begin(): void    │
│ + begin(): bool    │  ├────────────────────┤  │ + calculate(): float│
│ + readAll(): void  │  │ + find(): GapResult│  │ + reset(): void    │
│ + getAllData()     │  └────────────────────┘  └────────────────────┘
└────────────────────┘
                               │
                               ▼
                        ┌────────────────────┐
                        │      Actuator      │
                        ├────────────────────┤
                        │ - _steeringServo   │
                        │ - _escController   │
                        ├────────────────────┤
                        │ + begin(): void    │
                        │ + setSteering()    │
                        │ + setSpeed()       │
                        │ + stop(): void     │
                        └────────────────────┘

┌────────────────────┐  ┌────────────────────┐
│  <<header-only>>   │  │  <<header-only>>   │
│      Logger        │  │      Config        │
├────────────────────┤  ├────────────────────┤
│ + begin(): void    │  │ DEBUG_MODE         │
│ + print(): void    │  │ 各種定数           │
│ + println(): void  │  │                    │
│ (各種printXxx)     │  │                    │
└────────────────────┘  └────────────────────┘
```

### 2.2 データ構造

```cpp
// センサーデータ
struct SensorData {
    uint16_t distance;  // 測定距離（mm）
    bool valid;         // 有効性フラグ
};

// ギャップ情報
struct Gap {
    float start_angle;   // 開始角度（度）
    float end_angle;     // 終了角度（度）
    float center_angle;  // 中心角度（度）
    float width_angle;   // 幅（度）
    float min_distance;  // 最小距離（mm）
    bool valid;          // 有効フラグ
};

// ギャップ検出結果
struct GapResult {
    Gap best_gap;        // 最適ギャップ
    float target_angle;  // 目標角度（度）
    int num_gaps;        // 検出ギャップ数
    bool has_valid_gap;  // 有効ギャップの有無
};
```

---

## 3. クラス仕様

### 3.1 SensorReader クラス

**責務**: TCA9548A経由でVL53L1Xセンサーからの距離データ読み取りを管理

| メソッド | 引数 | 戻り値 | 説明 |
|---------|------|--------|------|
| `SensorReader()` | なし | - | コンストラクタ、内部データを初期化 |
| `begin()` | なし | `bool` | I2C初期化、全センサー初期化。成功時true |
| `readAll()` | なし | `void` | 全5センサーからデータを読み取り内部に保存 |
| `getSensorData(index)` | `uint8_t` | `SensorData` | 指定インデックスのセンサーデータを取得 |
| `getAllData()` | なし | `const SensorData*` | 全センサーデータ配列へのポインタを取得 |

**内部処理**:
- `_selectChannel(channel)`: TCA9548Aのチャンネルを切り替え
- 距離が4000mmを超える場合は4000mmにクリップ
- `MIN_VALID_DISTANCE` ～ `RELIABLE_RANGE`の範囲外は無効とする

### 3.2 GapFinder クラス

**責務**: センサーデータからFollow the Gapアルゴリズムで通過可能空間を検出

| メソッド | 引数 | 戻り値 | 説明 |
|---------|------|--------|------|
| `GapFinder()` | なし | - | コンストラクタ |
| `find(sensorData)` | `const SensorData*` | `GapResult` | ギャップ検出を実行し結果を返す |

**アルゴリズム**:
1. **障害物膨張**: 閾値以下の距離から安全マージンを差し引く
2. **ギャップ検出**: 連続する「開いた空間」をギャップとして記録
3. **スコアリング**: 距離・幅・前方優先度でスコア計算
4. **選択**: 最高スコアのギャップを選択

### 3.3 SteeringController クラス

**責務**: PD制御によるステアリング角度計算

| メソッド | 引数 | 戻り値 | 説明 |
|---------|------|--------|------|
| `SteeringController()` | なし | - | コンストラクタ |
| `begin()` | なし | `void` | 内部状態を初期化 |
| `calculate(gap)` | `const GapResult&` | `float` | PD制御でステアリング角度を計算 |
| `reset()` | なし | `void` | 内部状態をリセット |

**計算式**:
```
steering = Kp × target_angle + Kd × (target_angle - last_target_angle)
```

### 3.4 Actuator クラス

**責務**: サーボモーターとESCへのPWM出力制御

| メソッド | 引数 | 戻り値 | 説明 |
|---------|------|--------|------|
| `Actuator()` | なし | - | コンストラクタ |
| `begin()` | なし | `void` | Servo初期化、初期位置設定 |
| `setSteering(angle)` | `float` | `void` | ステアリング角度を設定（度） |
| `setSpeed(pulse_us)` | `uint16_t` | `void` | 速度パルス幅を設定（μs） |
| `calculateSpeedFromSteering(angle)` | `float` | `uint16_t` | ステアリング角度から速度パルスを計算 |
| `stop()` | なし | `void` | ESCを停止状態に設定 |

**重要**: DEBUG_MODE時はPWM出力を行わない

### 3.5 Logger クラス（静的クラス）

**責務**: DEBUG_MODEに応じたシリアル出力制御

| メソッド | 説明 |
|---------|------|
| `begin(baud)` | シリアル通信初期化（DEBUG_MODE時のみ） |
| `print(value)` | 値を出力（改行なし） |
| `println(value)` | 値を出力（改行あり） |
| `printSensorData(ch, dist, valid)` | センサーデータをフォーマット出力 |
| `printGapResult(...)` | ギャップ検出結果を出力 |
| `printSteering(angle)` | ステアリング角度を出力 |
| `printActuator(name, pulse)` | アクチュエーター情報を出力 |
| `printLoopTiming(loop_us, sensor_us)` | タイミング情報を出力 |

---

## 4. デバッグモード仕様

### 4.1 動作定義

| 項目 | DEBUG_MODE=true | DEBUG_MODE=false |
|------|-----------------|------------------|
| Serial.begin() | 実行（115200bps） | **実行しない** |
| シリアル出力 | 有効 | **全て無効** |
| Servo.attach() | **実行しない** | 実行 |
| PWM出力 | **行わない** | 行う |
| Serial1（HC-06） | 有効 | 有効 |

### 4.2 デバッグ出力フォーマット

```
S0:1234 | S1:567 | S2:890 | S3:456 | S4:789 | G:2 T:15.0° [C:12.3 W:40.0 D:1200] St:13.5 RR [Servo:1580] [ESC:1600] | T:28000us(S:25000us)
```

| フィールド | 説明 |
|-----------|------|
| `S0`～`S4` | 各センサー距離（mm）、無効時は`---` |
| `G` | 検出ギャップ数 |
| `T` | 目標角度（度） |
| `C/W/D` | 最適ギャップの中心角度/幅/最小距離 |
| `St` | ステアリング角度（度） |
| `L/R` | ステアリング方向インジケーター |
| `Servo` | サーボパルス幅（μs） |
| `ESC` | ESCパルス幅（μs） |
| `T(S)` | ループ時間/センサー読み取り時間（μs） |

### 4.3 起動時メッセージ

```
==========================================
  VL53L1X Follow the Gap + P Control
==========================================
Debug Mode: ON (No PWM)  または  OFF (PWM Active)
Measurement Interval: 35ms
Steering Kp: 0.9
Obstacle Threshold: 1500mm

--- Timing Configuration ---
  L1X Timing Budget: 20000 us
  L1X Inter-Measurement: 25 ms
  Loop Interval: 35 ms
----------------------------

=== VL53L1X Sensor Initialization ===
Sensor 0 (Ch0, -70.0deg)... OK
Sensor 1 (Ch1, -20.0deg)... OK
...
=== All VL53L1X sensors initialized ===

System ready!
```

---

## 5. 設定パラメータ仕様

### 5.1 デバッグ設定

| パラメータ | 型 | デフォルト値 | 説明 |
|-----------|-----|-------------|------|
| `DEBUG_MODE` | `bool` | `false` | デバッグモード有効/無効 |

### 5.2 ハードウェア設定

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `TCA9548A_ADDR` | `uint8_t` | `0x70` | I2Cマルチプレクサアドレス |
| `NUM_SENSORS` | `uint8_t` | `5` | センサー数 |
| `SENSOR_CHANNELS` | `uint8_t[5]` | `{0,1,2,3,4}` | マルチプレクサチャンネル |
| `SENSOR_ANGLES` | `float[5]` | `{-70,-20,0,20,70}` | 取付角度（度） |
| `SERVO_PIN` | `uint8_t` | `9` | サーボピン番号 |
| `ESC_PIN` | `uint8_t` | `10` | ESCピン番号 |

### 5.3 センサーパラメータ

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `MIN_VALID_DISTANCE` | `uint16_t` | `50` | 最小有効距離（mm） |
| `RELIABLE_RANGE` | `uint16_t` | `4000` | 最大有効距離（mm） |
| `L1X_TIMING_BUDGET_US` | `uint32_t` | `20000` | 測定時間（μs） |
| `L1X_INTER_MEASUREMENT_MS` | `uint32_t` | `25` | 測定間隔（ms） |

### 5.4 タイミング設定

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `MEASUREMENT_INTERVAL` | `unsigned long` | `35` | メインループ周期（ms） |

### 5.5 ステアリングパラメータ

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `MAX_STEERING_ANGLE` | `float` | `20.0` | 最大操舵角（度） |
| `STEERING_KP` | `float` | `0.9` | P制御ゲイン |
| `STEERING_KD` | `float` | `0.1` | D制御ゲイン |

### 5.6 Follow the Gap パラメータ

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `OBSTACLE_THRESHOLD` | `float` | `1500.0` | 障害物閾値（mm） |
| `OBSTACLE_INFLATION_RADIUS` | `float` | `200.0` | 膨張半径（mm） |
| `MIN_GAP_WIDTH_ANGLE` | `float` | `30.0` | 最小ギャップ幅（度） |
| `GAP_WEIGHT_DISTANCE` | `float` | `0.3` | 距離重み |
| `GAP_WEIGHT_WIDTH` | `float` | `0.4` | 幅重み |
| `GAP_WEIGHT_FORWARD` | `float` | `0.2` | 前方優先重み |

### 5.7 サーボ設定

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `SERVO_CENTER` | `uint16_t` | `1500` | 中央パルス幅（μs） |
| `SERVO_MIN` | `uint16_t` | `1200` | 最小パルス幅（μs）= 右最大 |
| `SERVO_MAX` | `uint16_t` | `1800` | 最大パルス幅（μs）= 左最大 |

### 5.8 ESC設定

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `ESC_STOP_US` | `uint16_t` | `1500` | 停止パルス幅（μs） |
| `ESC_MIN_US` | `uint16_t` | `1000` | 最小パルス幅（μs） |
| `ESC_MAX_US` | `uint16_t` | `2000` | 最大パルス幅（μs） |

### 5.9 速度連動設定

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `SPEED_STEERING_LINK_ENABLED` | `bool` | `true` | 速度連動有効 |
| `TOP_SPEED_US` | `uint16_t` | `1600` | 直進時最速（μs） |
| `CORNER_SPEED_US` | `uint16_t` | `1580` | コーナー時速度（μs） |
| `STEERING_DEADZONE` | `float` | `5.0` | デッドゾーン（度） |

### 5.10 安全設定

| パラメータ | 型 | 値 | 説明 |
|-----------|-----|-----|------|
| `EMERGENCY_FRONT_THRESHOLD` | `uint16_t` | `400` | 緊急停止距離（mm） |

---

## 6. メインループ仕様

### 6.1 処理フロー

```
loop() {
    1. Serial1入力チェック → 入力あれば停止して終了

    2. タイミングチェック（MEASUREMENT_INTERVAL経過？）
       └─ No → return

    3. センサーデータ読み取り
       └─ sensorReader.readAll()
       └─ Logger::printSensorData() × 5

    4. 緊急停止チェック
       └─ 前方センサー < EMERGENCY_FRONT_THRESHOLD → emergency_stop = true

    5. ギャップ検出
       └─ gap = gapFinder.find(sensorData)
       └─ Logger::printGapResult()

    6. ステアリング計算
       └─ steering_angle = steeringController.calculate(gap)
       └─ Logger::printSteering()

    7. アクチュエーター制御
       └─ if (emergency_stop)
          │   actuator.setSteering(0.0)
          │   actuator.stop()
          └─ else
              actuator.setSteering(steering_angle)
              speed = actuator.calculateSpeedFromSteering(steering_angle)
              actuator.setSpeed(speed)

    8. タイミング出力
       └─ Logger::printLoopTiming()
}
```

### 6.2 タイミング図

```
時間 (ms)    0    35   70   105  140  ...
             │    │    │    │    │
  Loop       ├────┼────┼────┼────┼────
             │    │    │    │    │
  センサー   █────█────█────█────█────  (約25ms/5センサー)
             │    │    │    │    │
  PWM出力    ○    ○    ○    ○    ○     (瞬時)
```

---

## 7. 安全機能仕様

### 7.1 緊急停止

**条件**: 前方センサー（インデックス2）の距離 < `EMERGENCY_FRONT_THRESHOLD`

**動作**:
- ステアリングを中央（0°）に設定
- ESCを停止パルス（1500μs）に設定

### 7.2 Bluetooth停止

**条件**: Serial1に任意のデータ受信

**動作**:
- ステアリングを中央に設定
- ESCを停止
- 無限ループで待機（手動リセットまで）

### 7.3 センサー異常時

**条件**: センサーデータが無効（valid=false）

**動作**:
- ギャップ検出時に障害物として扱う（安全側に倒す）
- 全センサー無効の場合は最後の有効な方向へ

---

## 8. 実装時の注意事項

### 8.1 Servoライブラリの制約
- `Servo.attach()`はPWMタイマーを占有する
- Arduino Nano R4ではD9/D10がServoで使用可能
- I2C通信との競合に注意

### 8.2 I2C通信
- TCA9548Aチャンネル切替後は10ms待機推奨
- Wire.setClock(400000)で高速モード使用

### 8.3 コンパイル時条件分岐
- `#if DEBUG_MODE` と `#if !DEBUG_MODE` を正しく使い分ける
- `DEBUG_MODE`は`#define`で定義（`const bool`ではない）

### 8.4 パルス幅変換
- ステアリング角度→パルス幅の変換は線形マッピング
- `map()`関数は整数演算のため精度に注意

---

## 9. ファイル構成

```
five_sensor_reader/
├── five_sensor_reader.ino  # メインスケッチ
├── Config.h                # 設定パラメータ一元管理
├── SensorReader.h          # センサー読み取りクラス（宣言）
├── SensorReader.cpp        # センサー読み取りクラス（実装）
├── GapFinder.h             # ギャップ検出クラス（宣言）
├── GapFinder.cpp           # ギャップ検出クラス（実装）
├── SteeringController.h    # ステアリング制御クラス（宣言）
├── SteeringController.cpp  # ステアリング制御クラス（実装）
├── Actuator.h              # アクチュエータークラス（宣言）
├── Actuator.cpp            # アクチュエータークラス（実装）
├── Logger.h                # ロガークラス（ヘッダーオンリー）
├── SPECIFICATION.md        # 本仕様書
└── README.md               # 使用方法
```

---

## 10. 付録: アルゴリズム詳細

### 10.1 Follow the Gap アルゴリズム

#### Step 1: 障害物膨張
```
for each sensor i:
    if distance[i] < OBSTACLE_THRESHOLD:
        inflated[i] = max(0, distance[i] - OBSTACLE_INFLATION_RADIUS)
    else:
        inflated[i] = distance[i]

    if not valid[i]:
        inflated[i] = 0  // 無効は障害物扱い
```

#### Step 2: ギャップ検出
```
isOpen[i] = (valid[i] && inflated[i] > OBSTACLE_THRESHOLD)

gapStart = -1
for i = 0 to NUM_SENSORS-1:
    if isOpen[i]:
        if gapStart < 0: gapStart = i
    else:
        if gapStart >= 0:
            recordGap(gapStart, i-1)
            gapStart = -1

if gapStart >= 0:
    recordGap(gapStart, NUM_SENSORS-1)
```

#### Step 3: ギャップスコアリング
```
distScore = gap.min_distance / RELIABLE_RANGE
widthScore = gap.width_angle / 140.0
forwardScore = 1.0 - abs(gap.center_angle) / 70.0

score = GAP_WEIGHT_DISTANCE × distScore
      + GAP_WEIGHT_WIDTH × widthScore
      + GAP_WEIGHT_FORWARD × forwardScore
```

### 10.2 PD制御

```
P項 = Kp × target_angle
D項 = Kd × (target_angle - last_target_angle)

steering = P項 + D項
steering = constrain(steering, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

last_target_angle = target_angle
```

### 10.3 速度連動制御

```
abs_angle = abs(steering_angle)

if abs_angle <= STEERING_DEADZONE:
    return TOP_SPEED_US

effective_angle = abs_angle - STEERING_DEADZONE
effective_max = MAX_STEERING_ANGLE - STEERING_DEADZONE
ratio = effective_angle / effective_max

// 二次関数で急角度ほど減速
speed_us = TOP_SPEED_US + (CORNER_SPEED_US - TOP_SPEED_US) × ratio²
```

---

## 改訂履歴

| 日付 | バージョン | 内容 |
|------|-----------|------|
| 2026-01-08 | 1.0 | 初版作成 |
