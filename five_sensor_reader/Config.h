/*
 * Config.h
 *
 * 定数・設定値の一元管理
 * Magic Numberを排除し、全ての定数を名前付きで管理
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// デバッグモード設定
// ============================================================================
#define DEBUG_MODE false  // true: デバッグ（PWMなし、シリアルあり）
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
// センサーパラメータ
// ============================================================================
const uint16_t MIN_VALID_DISTANCE = 50;        // 最小有効測定距離（mm）
const uint16_t RELIABLE_RANGE = 1200;          // 信頼できる測定範囲（mm）
const uint16_t SENSOR_ERROR_VALUE = 65535;     // センサーエラー時の値

// ============================================================================
// ステアリングパラメータ
// ============================================================================
const float MAX_STEERING_ANGLE = 30.0;         // 最大操舵角（度）

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

// ============================================================================
// サーボ・ESC パルス幅設定
// ============================================================================
// サーボ（ステアリング）
const uint16_t SERVO_CENTER = 1500;            // 中央位置（μs）
const uint16_t SERVO_MIN = 500;                // 最小パルス幅（μs）
const uint16_t SERVO_MAX = 2400;               // 最大パルス幅（μs）

// ESC（速度制御）
const float STOP_SPEED_PULSE = 1.5;            // 停止（ms）
const float BASE_SPEED_PULSE = 1.45;           // 基本速度（ms）
const float MAX_SPEED_PULSE = 1.4;             // 最大速度（ms）
const uint16_t ESC_MIN_US = 1000;              // ESC最小パルス（μs）
const uint16_t ESC_MAX_US = 2000;              // ESC最大パルス（μs）

// ============================================================================
// タイミング設定
// ============================================================================
const unsigned long MEASUREMENT_INTERVAL = 100; // 測定間隔（ms）= 10Hz

#endif // CONFIG_H
