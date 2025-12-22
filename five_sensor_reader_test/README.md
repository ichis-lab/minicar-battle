# five_sensor_reader テスト

`five_sensor_reader` Arduinoプロジェクトの単体テスト・結合テスト環境

## ディレクトリ構造

```
five_sensor_reader_test/
├── Dockerfile          # テスト用Docker環境
├── Makefile            # ビルド・テスト実行
├── README.md           # このファイル
├── src/                # テスト用にコピーしたソースコード
│   ├── Arduino.h       # Arduinoスタブ（millis等のモック）
│   ├── ArduinoMock.cpp # モック実装
│   ├── Config.h        # 設定定数
│   ├── Logger.h        # ログ出力スタブ
│   ├── SensorReader.h  # センサーデータ構造体
│   ├── PIDController.h/.cpp
│   ├── WallDetector.h/.cpp
│   └── SteeringController.h/.cpp
└── test/               # テストコード
    ├── test_pid_controller.cpp      # PID制御テスト
    ├── test_wall_detector.cpp       # 壁検出テスト
    ├── test_steering_controller.cpp # ステアリング制御テスト
    └── test_integration.cpp         # 結合テスト
```

## 実行方法

### Docker環境（デフォルト・推奨）

```bash
# 全テスト実行
make

# または
make test

# 特定のテストのみ実行
make test-pid        # PIDControllerテスト
make test-wall       # WallDetectorテスト
make test-steering   # SteeringControllerテスト
make test-integration # 結合テスト

# Dockerシェルでデバッグ
make shell
```

## テスト内容

### PIDController テスト（25件）

- P項（比例制御）: 正/負誤差、ゲイン倍率
- I項（積分制御）: 蓄積、アンチワインドアップ、リセット
- D項（微分制御）: 増減、ローパスフィルタ
- 不感帯（デッドバンド）
- 出力制限
- 初回実行・時間管理

### WallDetector テスト（20件）

- センサー有効性判定（範囲チェック）
- センサーペア差分チェック
- 壁距離・角度計算（幾何学）
- 壁検出結果（両壁/片壁/なし）
- エッジケース

### SteeringController テスト（25件）

- モード判定（両壁/左壁/右壁/なし）
- 両壁モード（距離PID）
- 片壁モード（角度PID + 安全距離）
- モード遷移とPIDリセット
- 出力クランプ
- 収束性

### 結合テスト（12件）

- センサー → 壁検出 → ステアリング パイプライン
- シナリオテスト（直線走路、コーナー進入）
- 時系列制御（収束性、積分蓄積）
- ロバスト性（ノイズ、モード切替）

## millis() のモック

テスト内で`millis()`の値を制御できます：

```cpp
#include "Arduino.h"

// 時刻を設定
ArduinoMock::setMillis(1000);  // 1000msに設定

// 時刻を進める
ArduinoMock::advanceMillis(40);  // 40ms進める

// リセット
ArduinoMock::resetMillis();  // 0に戻す
```

## オリジナルコードとの差分

| 項目 | オリジナル | テスト用 |
|------|-----------|---------|
| `#include <Arduino.h>` | Arduino SDK | `Arduino.h`スタブ |
| `millis()` | ハードウェア時刻 | モック関数 |
| `constrain()` | Arduinoマクロ | テンプレート関数 |
| `Logger` | シリアル出力 | 空実装 |

## カバレッジ

```bash
# カバレッジ付きでテスト実行
make coverage

# HTMLレポート生成（gcovr使用）
gcovr --html --html-details -o coverage.html
```

## トラブルシューティング

### `gtest not found`

```bash
# macOS
brew install googletest

# Ubuntu
sudo apt-get install libgtest-dev cmake
cd /usr/src/gtest
sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/
```

### Docker buildエラー

```bash
# キャッシュを無視して再ビルド
docker build --no-cache -t five_sensor_test .
```
