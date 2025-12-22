# テスト計画書: five_sensor_reader

## 概要

本ドキュメントは、`five_sensor_reader`プロジェクトの単体テスト・結合テスト計画を記載する。
特にPID制御に関わる箇所については、様々な壁状態を想定した網羅的なテストを設計する。

## テスト対象モジュール

```
┌─────────────────────────────────────────────────────────────┐
│                  five_sensor_reader.ino                      │
│                      (結合テスト)                             │
└───────────────┬─────────────────────────────────────────────┘
                │
    ┌───────────┼───────────┬───────────────┬────────────────┐
    │           │           │               │                │
    ▼           ▼           ▼               ▼                ▼
┌────────┐ ┌────────┐ ┌───────────┐ ┌────────────────┐ ┌──────────┐
│Config.h│ │Logger.h│ │SensorReader│ │WallDetector   │ │Actuator  │
│ (定数) │ │ (出力) │ │ (I/O依存)  │ │ (単体テスト)   │ │(I/O依存) │
└────────┘ └────────┘ └───────────┘ └───────┬────────┘ └──────────┘
                                            │
                                            ▼
                                   ┌────────────────┐
                                   │PIDController   │ ← 最重要テスト対象
                                   │ (単体テスト)   │
                                   └───────┬────────┘
                                            │
                                            ▼
                                   ┌────────────────────┐
                                   │SteeringController  │ ← 結合テスト
                                   │ (PID + WallDetect) │
                                   └────────────────────┘
```

## テストフレームワーク

### 推奨構成: PlatformIO + Unity + ArduinoFake

```
five_sensor_reader/
├── platformio.ini           # PlatformIOプロジェクト設定
├── src/                     # 本番コード（既存コードを移動）
│   ├── Config.h
│   ├── PIDController.h/.cpp
│   ├── WallDetector.h/.cpp
│   ├── SteeringController.h/.cpp
│   ├── SensorReader.h/.cpp
│   ├── Actuator.h/.cpp
│   └── main.cpp             # five_sensor_reader.ino → main.cpp
├── test/                    # テストコード
│   ├── test_pid_controller/
│   │   └── test_pid_controller.cpp
│   ├── test_wall_detector/
│   │   └── test_wall_detector.cpp
│   ├── test_steering_controller/
│   │   └── test_steering_controller.cpp
│   └── test_integration/
│       └── test_integration.cpp
└── lib/                     # 外部ライブラリ
```

### platformio.ini設定例

```ini
[env:native]
platform = native
test_framework = unity
build_flags =
    -D UNIT_TEST
    -D NATIVE_TEST
lib_deps =
    fabiobatsilern/ArduinoFake@^0.4

[env:arduino_nano_esp32]
platform = espressif32
board = arduino_nano_esp32
framework = arduino
test_framework = unity
```

---

## 1. 単体テスト

### 1.1 PIDController テスト（最重要）

**テストファイル**: `test/test_pid_controller/test_pid_controller.cpp`

#### 1.1.1 基本動作テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| PID-001 | 初期化テスト | コンストラクタで正しく初期化される | - | 全フィールドが初期値 |
| PID-002 | ゲイン設定テスト | setGains()でゲインが設定される | Kp=1.0, Ki=0.5, Kd=0.1 | 内部値が一致 |
| PID-003 | 出力制限テスト | setOutputLimits()で出力範囲が設定される | min=-30, max=30 | 範囲外の出力がクランプ |
| PID-004 | 積分制限テスト | setIntegralLimits()でワインドアップ防止 | min=-50, max=50 | 積分値が範囲内 |

#### 1.1.2 P項（比例制御）テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| PID-P01 | 正誤差でP項 | setpoint > measured | Kp=1.0, setpoint=100, measured=50 | P = 50.0 |
| PID-P02 | 負誤差でP項 | setpoint < measured | Kp=1.0, setpoint=50, measured=100 | P = -50.0 |
| PID-P03 | ゼロ誤差でP項 | setpoint == measured | Kp=1.0, setpoint=100, measured=100 | P = 0.0 |
| PID-P04 | ゲイン倍率 | Kpが正しく掛かる | Kp=0.5, error=100 | P = 50.0 |

#### 1.1.3 I項（積分制御）テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| PID-I01 | 積分蓄積 | 誤差が累積する | Ki=1.0, error=10, dt=0.04 × 5回 | I ≈ 2.0 |
| PID-I02 | アンチワインドアップ上限 | 積分値が上限でクランプ | max=50, 大きな誤差を継続 | integral ≤ 50 |
| PID-I03 | アンチワインドアップ下限 | 積分値が下限でクランプ | min=-50, 大きな負誤差 | integral ≥ -50 |
| PID-I04 | 積分リセット | reset()で積分がクリア | 積分蓄積後にreset() | integral = 0 |

#### 1.1.4 D項（微分制御）テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| PID-D01 | 誤差増加でD項正 | 誤差が増加している | prev_error=0 → error=10 | D > 0 |
| PID-D02 | 誤差減少でD項負 | 誤差が減少している | prev_error=10 → error=0 | D < 0 |
| PID-D03 | ローパスフィルタ効果 | ノイズが抑制される | 急激なerror変化 | filtered_Dは緩やかに変化 |
| PID-D04 | フィルタ係数0 | 完全フィルタ（変化なし） | alpha=0 | D項変化なし |
| PID-D05 | フィルタ係数1 | フィルタなし | alpha=1 | D項が即座に追従 |

#### 1.1.5 不感帯（デッドバンド）テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| PID-DB01 | 不感帯内で出力0 | 微小誤差が無視される | deadband=5, error=3 | output = 0 |
| PID-DB02 | 不感帯境界で動作 | 境界値での動作 | deadband=5, error=5 | output = 0 |
| PID-DB03 | 不感帯外で正常動作 | 大きな誤差は処理される | deadband=5, error=10 | output ≠ 0 |

#### 1.1.6 出力制限テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| PID-OL01 | 上限クランプ | 出力が上限を超えない | max=30, 大きな正誤差 | output = 30 |
| PID-OL02 | 下限クランプ | 出力が下限を下回らない | min=-30, 大きな負誤差 | output = -30 |
| PID-OL03 | 範囲内は透過 | 範囲内の値はそのまま | output=15, limits=±30 | output = 15 |

#### 1.1.7 初回実行・時間管理テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| PID-T01 | 初回は0を返す | 初回compute()は0 | 初回呼び出し | output = 0 |
| PID-T02 | dt=0での除算防止 | 同時刻で呼び出し | millis()同値 | エラーなし |
| PID-T03 | 正常dt計算 | 40ms間隔での計算 | 40ms間隔 | dt = 0.04 |

---

### 1.2 WallDetector テスト

**テストファイル**: `test/test_wall_detector/test_wall_detector.cpp`

#### 1.2.1 センサー有効性テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| WD-V01 | 有効範囲内 | 50-1200mm | distance=500 | valid=true |
| WD-V02 | 下限未満 | <50mm | distance=30 | valid=false |
| WD-V03 | 上限超過 | >1200mm | distance=1500 | valid=false |
| WD-V04 | 境界値下限 | 50mm | distance=50 | valid=true |
| WD-V05 | 境界値上限 | 1200mm | distance=1200 | valid=true |

#### 1.2.2 センサーペア差分テスト

| テストID | テスト名 | 説明 | 入力 | 期待出力 |
|----------|----------|------|------|----------|
| WD-D01 | 差分許容内 | <600mm差 | dist_far=600, dist_near=400 | 計算成功 |
| WD-D02 | 差分超過 | ≥600mm差 | dist_far=1000, dist_near=300 | valid=false |
| WD-D03 | 差分境界 | 600mm差 | dist_far=800, dist_near=200 | valid=false |

#### 1.2.3 壁距離計算テスト（幾何学）

```
センサー配置:
  Index 0: -70° (左側方)    Index 4: +70° (右側方)
  Index 1: -20° (左前方)    Index 3: +20° (右前方)
  Index 2:   0° (前方)

テストケース図:

  ケース1: 車体と平行な壁（左側300mm）
  ────────────────────────  壁
        ↑300mm
       [車体]

  ケース2: 車体に向かう壁（左側、角度+15°）
       ╲ 壁
        ╲
         ╲
       [車体]

  ケース3: 車体から離れる壁（左側、角度-15°）
         ╱ 壁
        ╱
       ╱
       [車体]
```

| テストID | テスト名 | 説明 | センサー入力 (S0, S1) | 期待結果 |
|----------|----------|------|------------------------|----------|
| WD-C01 | 平行壁（左） | 壁と車体が平行 | (320mm, 310mm) | angle≈0°, dist≈300mm |
| WD-C02 | 向かう壁（左） | 前方で壁に近づく | (400mm, 280mm) | angle>0°, dist計算あり |
| WD-C03 | 離れる壁（左） | 前方で壁から離れる | (280mm, 400mm) | angle<0°, dist計算あり |
| WD-C04 | 平行壁（右） | 右側の平行壁 | S3=310mm, S4=320mm | angle≈0°, dist≈300mm |
| WD-C05 | 極端に近い壁 | 50mm付近 | (60mm, 55mm) | valid計算あり |
| WD-C06 | 2点が同位置 | 計算不能 | 同一距離・同一角度 | valid=false |

#### 1.2.4 壁検出結果テスト

| テストID | テスト名 | 説明 | センサー状態 | 期待結果 |
|----------|----------|------|--------------|----------|
| WD-R01 | 両壁検出 | 左右有効 | 全センサー有効 | left_valid=true, right_valid=true |
| WD-R02 | 左壁のみ | 右側無効 | S3,S4=1500mm | left_valid=true, right_valid=false |
| WD-R03 | 右壁のみ | 左側無効 | S0,S1=1500mm | left_valid=false, right_valid=true |
| WD-R04 | 壁なし | 両側無効 | 全センサー遠い | left_valid=false, right_valid=false |
| WD-R05 | nullデータ | nullポインタ | nullptr | 初期値返却（valid=false） |

---

### 1.3 SteeringController テスト

**テストファイル**: `test/test_steering_controller/test_steering_controller.cpp`

#### 1.3.1 モード判定テスト

| テストID | テスト名 | 壁検出入力 | 期待モード |
|----------|----------|------------|------------|
| SC-M01 | 両壁モード | left=true, right=true | MODE_BOTH_WALLS |
| SC-M02 | 左壁モード | left=true, right=false | MODE_LEFT_WALL |
| SC-M03 | 右壁モード | left=false, right=true | MODE_RIGHT_WALL |
| SC-M04 | 壁なしモード | left=false, right=false | MODE_NO_WALLS |

#### 1.3.2 両壁モード（距離PID）テスト

```
テスト図:

  ケース1: 中央走行（理想状態）
  ═══════════════════════  左壁 (300mm)
           [車体]
  ═══════════════════════  右壁 (300mm)
  → ステアリング: 0°

  ケース2: 左寄り走行
  ═══════════════════════  左壁 (200mm)
        [車体]
  ═══════════════════════  右壁 (400mm)
  → ステアリング: 正（右へ）

  ケース3: 右寄り走行
  ═══════════════════════  左壁 (400mm)
             [車体]
  ═══════════════════════  右壁 (200mm)
  → ステアリング: 負（左へ）
```

| テストID | テスト名 | 左距離 | 右距離 | 期待ステアリング |
|----------|----------|--------|--------|------------------|
| SC-B01 | 中央走行 | 300mm | 300mm | ≈0° |
| SC-B02 | 左寄り補正 | 200mm | 400mm | >0° (右へ) |
| SC-B03 | 右寄り補正 | 400mm | 200mm | <0° (左へ) |
| SC-B04 | 大きな偏差 | 100mm | 500mm | 最大角付近 |
| SC-B05 | 不感帯内 | 295mm | 305mm | ≈0° (10mm不感帯) |

#### 1.3.3 左壁モード（角度PID）テスト

```
テスト図:

  ケース1: 壁と平行
  ════════════════  左壁
       →
      [車体] 進行方向→
  → ステアリング: 0°

  ケース2: 壁に向かっている (angle > 0)
       ╲ 左壁
        ╲
      [車体] ↗
  → ステアリング: 正（右へ逃げる）

  ケース3: 壁から離れている (angle < 0)
       ╱ 左壁
      ╱
      [車体] ↘
  → ステアリング: 負（左へ戻る）
```

| テストID | テスト名 | 左壁角度 | 左壁距離 | 期待ステアリング |
|----------|----------|----------|----------|------------------|
| SC-L01 | 壁と平行 | 0° | 400mm | ≈0° |
| SC-L02 | 壁に向かう | +10° | 400mm | >0° (右へ) |
| SC-L03 | 壁から離れる | -10° | 400mm | <0° (左へ) |
| SC-L04 | 安全距離違反 | 0° | 200mm | >0° (距離回避) |
| SC-L05 | 大角度+近距離 | +15° | 150mm | 大きく右へ |

#### 1.3.4 右壁モード（角度PID）テスト

| テストID | テスト名 | 右壁角度 | 右壁距離 | 期待ステアリング |
|----------|----------|----------|----------|------------------|
| SC-R01 | 壁と平行 | 0° | 400mm | ≈0° |
| SC-R02 | 壁に向かう | +10° | 400mm | <0° (左へ) |
| SC-R03 | 壁から離れる | -10° | 400mm | >0° (右へ) |
| SC-R04 | 安全距離違反 | 0° | 200mm | <0° (距離回避) |
| SC-R05 | 大角度+近距離 | +15° | 150mm | 大きく左へ |

#### 1.3.5 壁なしモードテスト

| テストID | テスト名 | 入力 | 期待ステアリング |
|----------|----------|------|------------------|
| SC-N01 | 壁なしで直進 | valid=false両方 | 0° |

#### 1.3.6 モード遷移テスト

| テストID | テスト名 | 遷移 | 確認ポイント |
|----------|----------|------|--------------|
| SC-T01 | 両壁→左壁 | 右壁消失 | PIDリセット発生 |
| SC-T02 | 左壁→両壁 | 右壁出現 | PIDリセット発生 |
| SC-T03 | 両壁→壁なし | 両壁消失 | PIDリセット発生 |
| SC-T04 | 同モード継続 | 変化なし | PIDリセットなし |

#### 1.3.7 出力クランプテスト

| テストID | テスト名 | 計算結果 | 期待出力 |
|----------|----------|----------|----------|
| SC-CL01 | 上限クランプ | +50° | +30° |
| SC-CL02 | 下限クランプ | -50° | -30° |
| SC-CL03 | 範囲内透過 | +15° | +15° |

---

### 1.4 Actuator テスト（ハードウェア依存部）

**テストファイル**: `test/test_actuator/test_actuator.cpp`

※ モックを使用してI/O依存を排除

#### 1.4.1 ステアリング変換テスト

| テストID | テスト名 | 入力角度 | 期待パルス幅(μs) |
|----------|----------|----------|------------------|
| ACT-S01 | 中央 | 0° | 1500 |
| ACT-S02 | 右最大 | +90° | 600 (SERVO_MIN) |
| ACT-S03 | 左最大 | -90° | 2400 (SERVO_MAX) |
| ACT-S04 | 右30° | +30° | ≈1200 |
| ACT-S05 | 左30° | -30° | ≈1800 |
| ACT-S06 | 上限超過 | +100° | 600 (クランプ) |
| ACT-S07 | 下限超過 | -100° | 2400 (クランプ) |

#### 1.4.2 速度変換テスト

| テストID | テスト名 | 入力(ms) | 期待パルス幅(μs) |
|----------|----------|----------|------------------|
| ACT-E01 | 停止 | 1.5ms | 1500 |
| ACT-E02 | 基本速度 | 1.45ms | 1450 |
| ACT-E03 | 最大速度 | 1.0ms | 1000 |
| ACT-E04 | 後退 | 2.0ms | 2000 |
| ACT-E05 | 下限クランプ | 0.5ms | 1000 |
| ACT-E06 | 上限クランプ | 2.5ms | 2000 |

---

## 2. 結合テスト

### 2.1 WallDetector + SteeringController 結合

**テストファイル**: `test/test_integration/test_wall_steering.cpp`

#### 2.1.1 シナリオテスト

```
シナリオ1: 直線走路（両壁あり）
  ═══════════════════════════════════════
       [車体] →→→→→→→→→→→→→→→→→
  ═══════════════════════════════════════

  入力: 5つのセンサー値（両側検出可能な距離）
  期待: MODE_BOTH_WALLS、中央維持のステアリング

シナリオ2: 左コーナー進入
  ═══════════════════════════════════════
       [車体] →→→→→→╲
  ═════════════════╲
                    ╲  右壁が開く
                     ╲

  入力: 右センサーが遠距離に変化
  期待: MODE_LEFT_WALL に遷移、左壁追従

シナリオ3: 壁からの距離変化（動的）
  時刻t0: 両壁300mm
  時刻t1: 左壁280mm（左に寄った）
  時刻t2: 左壁260mm（さらに左に）

  期待: PIDの積分項が蓄積し、ステアリングが徐々に増加
```

| テストID | シナリオ | センサー入力 | 期待動作 |
|----------|----------|--------------|----------|
| INT-01 | 直線両壁走行 | 全センサー300mm | MODE_BOTH, steering≈0 |
| INT-02 | 直線左寄り | 左200mm, 右400mm | steering正（右補正） |
| INT-03 | 左コーナー進入 | 右センサー遠距離化 | MODE_LEFT_WALLに遷移 |
| INT-04 | 右コーナー進入 | 左センサー遠距離化 | MODE_RIGHT_WALLに遷移 |
| INT-05 | 緊急回避 | 左壁150mm接近 | 大きく右へステア |

### 2.2 センサー → 壁検出 → ステアリング → アクチュエーター

**テストファイル**: `test/test_integration/test_full_pipeline.cpp`

```
パイプラインテスト:

  SensorData[5]
       │
       ▼
  WallDetector.detect()
       │
       ▼
  WallDetection {left_valid, right_valid, distances, angles}
       │
       ▼
  SteeringController.calculate()
       │
       ▼
  steering_angle (float, -30 to +30)
       │
       ▼
  Actuator.setSteering()
       │
       ▼
  PWM pulse width (600-2400 μs)
```

| テストID | 入力センサー値 | 期待PWM出力 |
|----------|----------------|-------------|
| FP-01 | 全300mm（中央） | ≈1500μs |
| FP-02 | 左200mm, 右400mm | <1500μs（右へ） |
| FP-03 | 左400mm, 右200mm | >1500μs（左へ） |
| FP-04 | 右センサー遠距離 | 左壁追従のPWM |

### 2.3 時系列制御テスト（PID連続動作）

**テストファイル**: `test/test_integration/test_pid_sequence.cpp`

```
テストシナリオ: 左右への揺れの収束

時刻    左壁距離  右壁距離  ステアリング
─────────────────────────────────────
t=0     300       300       0.0
t=40ms  280       320       +3.2  (右へ)
t=80ms  290       310       +1.6  (収束中)
t=120ms 295       305       +0.4  (ほぼ中央)
t=160ms 298       302       ≈0    (収束)
```

| テストID | テスト名 | 確認ポイント |
|----------|----------|--------------|
| SEQ-01 | 収束性 | ステアリングが0に収束する |
| SEQ-02 | オーバーシュートなし | 振動なく収束 |
| SEQ-03 | モード切替収束 | 切替後も安定して収束 |

---

## 3. テスト実装のポイント

### 3.1 Arduinoライブラリのモック化

```cpp
// ArduinoFakeを使用したモック例
#include <ArduinoFake.h>

using namespace fakeit;

void test_pid_compute_with_mock_time() {
    // millis()のモック
    When(Method(ArduinoFake(), millis))
        .Return(0)      // 1回目: t=0
        .Return(40)     // 2回目: t=40ms
        .Return(80);    // 3回目: t=80ms

    PIDController pid;
    pid.begin(1.0, 0.0, 0.0);  // Pのみ

    float output1 = pid.compute(100.0, 50.0);  // 初回: 0を返す
    float output2 = pid.compute(100.0, 50.0);  // 2回目: P=50

    TEST_ASSERT_EQUAL_FLOAT(0.0, output1);
    TEST_ASSERT_EQUAL_FLOAT(50.0, output2);
}
```

### 3.2 WallDetector用のセンサーデータ生成

```cpp
// テスト用センサーデータ生成ヘルパー
SensorData createSensorData(uint16_t distances[5], bool valids[5] = nullptr) {
    static SensorData data[5];
    for (int i = 0; i < 5; i++) {
        data[i].distance = distances[i];
        data[i].valid = valids ? valids[i] : true;
        data[i].status = 0;
    }
    return data;
}

// 使用例: 両壁300mm
void test_wall_detection_both_walls() {
    uint16_t distances[] = {320, 310, 1000, 310, 320};  // 左右センサー近距離
    SensorData* data = createSensorData(distances);

    WallDetector detector;
    WallDetection result = detector.detect(data);

    TEST_ASSERT_TRUE(result.left_valid);
    TEST_ASSERT_TRUE(result.right_valid);
    TEST_ASSERT_FLOAT_WITHIN(50.0, 300.0, result.left_distance);
    TEST_ASSERT_FLOAT_WITHIN(50.0, 300.0, result.right_distance);
}
```

### 3.3 PID連続動作テスト

```cpp
// PIDの時間経過をシミュレート
void test_pid_integration_accumulation() {
    unsigned long mock_time = 0;
    When(Method(ArduinoFake(), millis)).AlwaysDo([&mock_time]() {
        return mock_time;
    });

    PIDController pid;
    pid.begin(0.0, 1.0, 0.0);  // Iのみ
    pid.setIntegralLimits(-100, 100);

    // 初回呼び出し
    pid.compute(100, 0);  // error=100

    // 40msごとに5回呼び出し（200ms）
    for (int i = 0; i < 5; i++) {
        mock_time += 40;
        float output = pid.compute(100, 0);  // error=100継続
    }

    // 積分値: 100 * 0.04 * 5 = 20.0
    TEST_ASSERT_FLOAT_WITHIN(1.0, 20.0, pid.getIntegral());
}
```

---

## 4. テスト優先度

### 最優先（P0）
1. **PIDController 全テスト** - システムの核心
2. **WallDetector 壁距離計算テスト** - 正しい幾何計算が必須
3. **SteeringController モード判定・両壁モード** - 最も使用頻度が高い

### 高優先（P1）
1. SteeringController 片壁モード（左・右）
2. SteeringController モード遷移
3. 結合テスト（センサー→ステアリング）

### 中優先（P2）
1. Actuator PWM変換
2. 時系列制御テスト
3. エッジケース（境界値、null入力）

---

## 5. テスト実行手順

### PlatformIO環境での実行

```bash
# 全テスト実行（ネイティブ環境）
pio test -e native

# 特定のテストのみ実行
pio test -e native -f test_pid_controller

# 実機テスト（Arduino接続時）
pio test -e arduino_nano_esp32

# 詳細出力
pio test -e native -v
```

### テストカバレッジ（gcov使用時）

```bash
# カバレッジ付きビルド
pio test -e native --coverage

# レポート生成
gcovr --html --html-details -o coverage.html
```

---

## 6. 今後の拡張

1. **パラメトリックテスト**: 様々なPIDゲインでの安定性検証
2. **ファジングテスト**: ランダム入力での異常動作検出
3. **ハードウェアインザループ(HIL)テスト**: 実センサー接続での検証
4. **長時間テスト**: 積分項のドリフト検証

---

## 付録A: センサー幾何学

```
センサー配置と座標系:

        y軸（進行方向）
         ↑
         │
   -70°  │ 0°  +70°
     ╲   │   ╱
      ╲  │  ╱
   -20° ╲│╱ +20°
─────────┼────────→ x軸
        (0,0)
        車体中心

センサー番号:
  Index 0: θ = -70° (左側方)
  Index 1: θ = -20° (左前方)
  Index 2: θ =   0° (前方)
  Index 3: θ = +20° (右前方)
  Index 4: θ = +70° (右側方)

座標変換:
  x = distance × sin(θ)
  y = distance × cos(θ)
```

## 付録B: PID制御ブロック図

```
                    ┌─────────────────────────────────────────────┐
                    │                PIDController                 │
                    │                                              │
setpoint ──────────►│  ┌───────┐                                  │
                    │  │ error │◄─────────────────┐               │
measured ──────────►│  │  = sp │                  │               │
                    │  │  - mv │                  │               │
                    │  └───┬───┘                  │               │
                    │      │                      │               │
                    │      ▼                      │               │
                    │  ┌───────┐    ┌─────────┐   │               │
                    │  │deadband│───►│ P = Kp*e│───┼───┐          │
                    │  └───┬───┘    └─────────┘   │   │          │
                    │      │                      │   │          │
                    │      │        ┌─────────┐   │   │ ┌──────┐ │
                    │      ├───────►│integral │───┼───┼─►│  +   │─┼──► output
                    │      │        │ += e*dt │   │   │ │      │ │
                    │      │        │ clamp() │   │   │ │      │ │
                    │      │        └─────────┘   │   │ └──────┘ │
                    │      │                      │   │    ▲     │
                    │      │        ┌─────────┐   │   │    │     │
                    │      └───────►│derivative│──┼───┘    │     │
                    │               │ = de/dt │   │ clamp(out)   │
                    │               │ filter()│   │        │     │
                    │               └─────────┘   │        │     │
                    │                              │   output_limits
                    └─────────────────────────────────────────────┘
```
