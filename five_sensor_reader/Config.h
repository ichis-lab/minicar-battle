/*
 * Config.h
 *
 * 定数・設定値の一元管理
 * シンプル状態ベース制御用
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// デバッグモード設定
// ============================================================================
#define DEBUG_MODE false   // true: デバッグ（PWMなし、シリアルあり）
                          // false: 実機（PWMあり、シリアルなし）

// ============================================================================
// ハードウェア設定
// ============================================================================
// TCA9548A I2Cマルチプレクサ
const uint8_t TCA9548A_ADDR = 0x70;

// センサー設定
const uint8_t NUM_SENSORS = 5;
const uint8_t SENSOR_CHANNELS[NUM_SENSORS] = {0, 1, 2, 3, 4};
const float SENSOR_ANGLES[NUM_SENSORS] = {-70.0, -20.0, 0.0, 20.0, 70.0};

// PWM出力ピン
const uint8_t SERVO_PIN = 9;   // ステアリングサーボ
const uint8_t ESC_PIN = 10;    // ESC（モーター制御）

// ============================================================================
// VL53L1X センサーパラメータ
// ============================================================================
const uint16_t MIN_VALID_DISTANCE = 50;        // 最小有効測定距離（mm）
const uint16_t RELIABLE_RANGE = 4000;          // 信頼できる測定範囲（mm）L1Xは最大4m

// VL53L1X タイミング設定
const uint32_t L1X_TIMING_BUDGET_US = 50000;   // 測定時間（μs）50ms
const uint32_t L1X_INTER_MEASUREMENT_MS = 50;  // 測定間隔（ms）

// ============================================================================
// タイミング設定
// ============================================================================
const unsigned long MEASUREMENT_INTERVAL = 60;  // 60ms（L1X測定間隔以上を推奨）

// ============================================================================
// ステアリング基本パラメータ
// ============================================================================
const float MAX_STEERING_ANGLE = 30.0;         // 最大操舵角（度）

// ============================================================================
// シンプル制御パラメータ
// ============================================================================

// --- 正面壁検出閾値 ---
const uint16_t CORNER_THRESHOLD = 800;         // コーナリング開始（mm）
const uint16_t EMERGENCY_THRESHOLD = 300;      // 緊急回避開始（mm）

// --- 側壁回避 ---
const uint16_t MIN_SIDE_DISTANCE = 200;        // 側壁最低距離（mm）
const float EMERGENCY_AVOID_ANGLE = 20.0;      // 緊急回避ステアリング（度）

// --- 右壁追従 ---
const uint16_t TARGET_WALL_DISTANCE = 600;     // 右壁との目標距離（mm）
const uint16_t WALL_TOLERANCE = 150;           // 許容範囲（mm）
const float WALL_AVOID_ANGLE = 8.0;            // 壁回避ステアリング（度）
const float WALL_APPROACH_ANGLE = 5.0;         // 壁接近ステアリング（度）

// --- コーナリング ---
const float CORNER_ANGLE = 22.0;               // コーナリングステアリング（度）

// ============================================================================
// サーボ・ESC パルス幅設定
// ============================================================================
// サーボ（ステアリング）
const uint16_t SERVO_CENTER = 1510;           // 中央位置（μs）
const uint16_t SERVO_MIN = 600;               // 最小パルス幅（μs）
const uint16_t SERVO_MAX = 2400;              // 最大パルス幅（μs）

// ESC（速度制御）
const float STOP_SPEED_PULSE = 1.5;           // 停止（ms）
const float BASE_SPEED_PULSE = 1.45;          // 基本速度（ms）
const uint16_t ESC_MIN_US = 1000;             // ESC最小パルス（μs）
const uint16_t ESC_MAX_US = 2000;             // ESC最大パルス（μs）

#endif // CONFIG_H
