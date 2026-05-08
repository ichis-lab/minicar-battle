/*
 * Config.h
 *
 * Centralized configuration values.
 * All constants are named here to avoid magic numbers.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Run mode
// ============================================================================
// Mode definitions:
//   0: MODE_DEBUG      - debug only (no PWM, serial enabled)
//   1: MODE_PRODUCTION - race run (PWM enabled, serial disabled)
//   2: MODE_DEBUG_RUN  - debug run (PWM enabled, serial enabled)
#define MODE_DEBUG 0
#define MODE_PRODUCTION 1
#define MODE_DEBUG_RUN 2

#define RUN_MODE MODE_DEBUG // select run mode here

// Feature toggles derived from RUN_MODE
#define ENABLE_SERIAL (RUN_MODE == MODE_DEBUG || RUN_MODE == MODE_DEBUG_RUN)
#define ENABLE_PWM (RUN_MODE == MODE_PRODUCTION || RUN_MODE == MODE_DEBUG_RUN)

// ============================================================================
// Hardware
// ============================================================================
// TCA9548A I2C multiplexer
const uint8_t TCA9548A_ADDR = 0x70;

// Sensor layout
const uint8_t NUM_SENSORS = 7;
const uint8_t SENSOR_CHANNELS[NUM_SENSORS] = {0, 1, 2, 3, 4, 5, 6};
// Physical mounting is +/-60deg, but these tuned values produced the winning 17.5s final (3 laps).
const float SENSOR_ANGLES[NUM_SENSORS] = {-50.0, -30.0, -15.0, 0.0, 15.0, 30.0, 50.0};
const uint8_t FRONT_SENSOR_INDEX = 3;  // index of the front-facing sensor (0 deg)

// PWM output pins
const uint8_t SERVO_PIN = 9;  // steering servo
const uint8_t ESC_PIN = 10;   // ESC (motor)

// ============================================================================
// Sensor parameters
// ============================================================================
const uint16_t MIN_VALID_DISTANCE = 50;  // minimum valid range (mm)
const uint16_t RELIABLE_RANGE = 4000;    // reliable measurement range (mm); VL53L1X max is 4m
// VL53L1X timing
const uint32_t L1X_TIMING_BUDGET_US = 33000;   // measurement budget (us)
const uint32_t L1X_INTER_MEASUREMENT_MS = 40;  // inter-measurement period (ms)

// ============================================================================
// Loop timing
// 40ms is the floor and maximizes measurements per second.
// 40-60ms is the VL53L1X medium-mode range (~3m effective).
// ============================================================================
const unsigned long MEASUREMENT_INTERVAL = 40;  // must be >= L1X inter-measurement period


// ============================================================================
// Steering
// ============================================================================
const float MAX_STEERING_ANGLE = 30.0;  // max steering angle (deg)

// ============================================================================
// Gap detection
// Tested 0-200; no clear effect on yaw oscillation.
// ============================================================================
const float FARTHEST_HYSTERESIS = 100.0;  // hysteresis for farthest-sensor switching (mm); tested 0-300

// ============================================================================
// Pure Pursuit
// Formula: steering_angle = atan2(2 * L * sin(alpha), Ld)
//   L:     wheelbase (mm)
//   alpha: angle to the target point (rad)
//   Ld:    lookahead distance (mm) = front sensor distance - body length
// ============================================================================
const float WHEELBASE_MM = 210.0;       // wheelbase (mm), MF-01X
// Manually tuned. Sweep history: SPEED 1640 -> 900, 1660 -> 1000, 1680 -> 1100, > 1680 -> 1200.
const float LOOKAHEAD_OFFSET_MM = 1100.0;

// ============================================================================
// Safety
// ============================================================================
const uint16_t EMERGENCY_FRONT_THRESHOLD = 400;  // front emergency-stop threshold (mm)

// Auto-stop timeout (debug aid).
// 0 disables it; otherwise the car stops after this many seconds.
const unsigned long AUTO_STOP_SECONDS = 20;

// ============================================================================
// Servo / ESC pulse widths
// ============================================================================
// Steering servo. New servo nominal range is 1200(R)-1500(C)-1800(L);
// values below are calibrated for the current install.
const uint16_t SERVO_CENTER = 1415;  // center (us)
const uint16_t SERVO_MIN = 1115;     // min pulse (us) = full right
const uint16_t SERVO_MAX = 1715;     // max pulse (us) = full left

// ESC
const uint16_t ESC_STOP_US = 1500;   // neutral / stop (us)
const uint16_t ESC_MIN_US = 1000;    // min ESC pulse (us)
const uint16_t ESC_MAX_US = 2000;    // max ESC pulse (us)
const uint16_t SPEED_US = 1680;      // cruise pulse (us)

// ============================================================================
// Math constants
// ============================================================================
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f  // PI / 180.0
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232f  // 180.0 / PI
#endif

// Precomputed sin values for the angular gaps between sensors
const float SIN_15_DEG = 0.2588190451f;  // sin(15deg)
const float SIN_20_DEG = 0.3420201433f;  // sin(20deg)

#endif  // CONFIG_H
