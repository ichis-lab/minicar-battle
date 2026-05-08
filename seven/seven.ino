/*
 * seven.ino
 *
 * Follow-the-Gap + Pure Pursuit control over 7 VL53L1X sensors.
 *
 * Wiring:
 * - TCA9548A I2C multiplexer (address 0x70)
 * - VL53L1X sensors connect to multiplexer channels 0-6
 * - Steering servo on pin 9
 * - ESC on pin 10
 *
 * Usage:
 * 1. Open this sketch in the Arduino IDE.
 * 2. Install the required libraries:
 *    - VL53L1X (Pololu)
 *    - Servo (Arduino built-in)
 * 3. Set RUN_MODE in Config.h:
 *    - MODE_DEBUG: debug only (no PWM, serial enabled)
 *    - MODE_PRODUCTION: race run (PWM enabled, serial disabled)
 *    - MODE_DEBUG_RUN: debug run (PWM enabled, serial enabled)
 * 4. Flash to an Arduino Nano R4.
 */

#include "Actuator.h"
#include "Config.h"
#include "GapFinder.h"
#include "Logger.h"
#include "SensorReader.h"
#include "SteeringController.h"

// ============================================================================
// Globals
// ============================================================================
SensorReader sensorReader;
GapFinder gapFinder;
SteeringController steeringController;
Actuator actuator;

// ============================================================================
// setup
// ============================================================================
void setup() {
    // Logger init (115200 baud for detailed telemetry)
    Logger::begin(115200);

    Logger::println("==============================================");
    Logger::println("  VL53L1X Follow the Gap + Pure Pursuit");
    Logger::println("==============================================");
    Logger::print("Run Mode: ");
#if RUN_MODE == MODE_DEBUG
    Logger::println("DEBUG (No PWM, Serial ON)");
#elif RUN_MODE == MODE_PRODUCTION
    Logger::println("PRODUCTION (PWM ON, Serial OFF)");
#else
    Logger::println("DEBUG_RUN (PWM ON, Serial ON)");
#endif
    Logger::print("Measurement Interval: ");
    Logger::print(MEASUREMENT_INTERVAL);
    Logger::println("ms");
    Logger::print("Wheelbase: ");
    Logger::print(WHEELBASE_MM, 0);
    Logger::println("mm");
    Logger::println("Lookahead: Front sensor distance");
    Logger::println();

    // Print timing config summary
    Logger::printTimingConfig();
    Logger::println();

    // Sensor init
    if (!sensorReader.begin()) {
        Logger::println("ERROR: Sensor initialization failed!");
        while (1) {
            delay(100);
        }
    }

    // Steering controller init
    steeringController.begin();

    // Actuator init
    actuator.begin();

    // Wait for ESC arming
    Logger::println("Waiting 3 seconds for ESC arming...");
    delay(3000);

    Logger::println();
    Logger::println("System ready!");
    Logger::println();
}

// ============================================================================
// loop
// ============================================================================
void loop() {
    static unsigned long lastMeasurement = 0;
    static unsigned long startTime = 0;
    static bool firstRun = true;
    unsigned long currentTime = millis();

    // Record start time on the first iteration
    if (firstRun) {
        startTime = currentTime;
        firstRun = false;
    }

    // Auto-stop on timeout
    if (AUTO_STOP_SECONDS > 0 &&
        (currentTime - startTime) >= (AUTO_STOP_SECONDS * 1000UL)) {
        actuator.setSteering(0.0);
        actuator.stop();
        Logger::println("AUTO STOP: Timeout reached");
        while (1) {
            delay(1000);
        }
    }

    // Run measurement and control on a fixed cadence
    if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
        lastMeasurement += MEASUREMENT_INTERVAL;

        // Start loop timer
        unsigned long loopStartUs = micros();

        // =========================================================================
        // Phase 1: read sensors
        // =========================================================================
        unsigned long sensorStartUs = micros();
        sensorReader.readAll();
        unsigned long sensorEndUs = micros();
        unsigned long sensorElapsedUs = sensorEndUs - sensorStartUs;

        const SensorData* sensorData = sensorReader.getAllData();

        // Debug: print sensor readings
        for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
            Logger::printSensorData(SENSOR_CHANNELS[i], sensorData[i].distance,
                                    sensorData[i].valid);
            if (i < NUM_SENSORS - 1) {
                Logger::print(" | ");
            }
        }

        // =========================================================================
        // Phase 2: emergency stop check (front obstacle)
        // =========================================================================
        bool emergency_stop = false;
        if (sensorData[FRONT_SENSOR_INDEX].valid &&
            sensorData[FRONT_SENSOR_INDEX].distance < EMERGENCY_FRONT_THRESHOLD) {
            emergency_stop = true;
            Logger::print(" | EMERGENCY!");
        }

        // =========================================================================
        // Phase 3: gap detection (farthest sensor + neighbors)
        // =========================================================================
        GapResult gap = gapFinder.find(sensorData);

        // Debug: print gap result (Ld is derived from the front sensor distance)
        float front_distance = sensorData[FRONT_SENSOR_INDEX].valid
                               ? sensorData[FRONT_SENSOR_INDEX].distance
                               : 0.0f;
        Logger::printGapResult(gap.target_angle, front_distance);

        // =========================================================================
        // Phase 4: steering angle (Pure Pursuit)
        // =========================================================================
        float steering_angle = steeringController.calculate(gap, sensorData);

        // Debug: print target/steering angles
        Logger::printSteering(steering_angle);

        // =========================================================================
        // Phase 5: actuators
        // =========================================================================
        if (emergency_stop) {
            // Emergency stop: center the steering and cut power.
            actuator.setSteering(0.0);
            actuator.stop();
        } else {
            // Normal cruise.
            actuator.setSteering(steering_angle);
            actuator.setSpeed(SPEED_US);
        }

        // Loop timer end + print
        unsigned long loopEndUs = micros();
        unsigned long loopElapsedUs = loopEndUs - loopStartUs;
        Logger::printLoopTiming(loopElapsedUs, sensorElapsedUs);

        Logger::println("");
    }
}
