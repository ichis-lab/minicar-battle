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
const uint16_t MAX_SENSOR_DIFF = 600;          // センサーペア間の最大許容差（mm）
const uint16_t SENSOR_ERROR_VALUE = 65535;     // センサーエラー時の値

// ============================================================================
// タイミング設定（更新）/ Timing Settings (Updated)
// ============================================================================
const unsigned long MEASUREMENT_INTERVAL = 40;  // 40ms = 25Hz (was 100ms = 10Hz)
const uint32_t SENSOR_TIMING_BUDGET = 20000;    // 20ms per sensor

// ============================================================================
// ステアリングパラメータ
// ============================================================================
const float MAX_STEERING_ANGLE = 30.0;         // 最大操舵角（度）

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

// 左側センサーの最小距離制約（衝突回避）
const uint16_t MIN_LEFT_DISTANCE = 300;    // 左側最小距離（mm）= 30cm
const float LEFT_AVOID_GAIN = 0.1;         // 回避補正ゲイン

// 緊急回避パラメータ / Emergency avoidance parameters
const uint16_t EMERGENCY_FRONT_THRESHOLD = 200;   // 前方緊急閾値(mm)

// ============================================================================
// 速度制御パラメータ / Speed Control Parameters
// ============================================================================
const float SPEED_KP = 0.001;         // 速度PID比例ゲイン / Speed PID proportional gain
const float MIN_SPEED_PULSE = 1.48;   // 最小速度 / Minimum speed (ms)
const float MAX_SPEED_PULSE = 1.40;   // 最大速度 / Maximum speed (ms)
const float CORNER_SPEED_PULSE = 1.47; // コーナー速度 / Corner speed (ms)

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
const uint16_t ESC_MIN_US = 1000;              // ESC最小パルス（μs）
const uint16_t ESC_MAX_US = 2000;              // ESC最大パルス（μs）

#endif // CONFIG_H
