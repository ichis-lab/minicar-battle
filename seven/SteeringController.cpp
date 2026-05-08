/*
 * SteeringController.cpp
 *
 * Pure Pursuit on the GapFinder target.
 *
 * Formula: steering = atan2(2 * L * sin(alpha), Ld)
 *   L:     wheelbase (mm)
 *   alpha: angle to the target point (rad)
 *   Ld:    lookahead distance (mm) = front sensor distance - LOOKAHEAD_OFFSET_MM
 */

#include "SteeringController.h"

#include <math.h>

SteeringController::SteeringController() {}

void SteeringController::begin() {
    // Pure Pursuit is stateless; nothing to initialize.
}

float SteeringController::calculate(const GapResult& gap, const SensorData* sensorData) {
    // Treat GapFinder's output as the polar coordinates of the target point.
    // alpha: angle to the target.
    // Ld:    lookahead distance, derived from the front sensor.

    float alpha_deg = gap.target_angle;

    // Subtract the offset from the front sensor distance to get Ld.
    float Ld_mm = sensorData[FRONT_SENSOR_INDEX].valid
                  ? sensorData[FRONT_SENSOR_INDEX].distance - LOOKAHEAD_OFFSET_MM
                  : 1000.0f;  // fallback when the front sensor is invalid

    // Avoid divide-by-zero / extreme values.
    if (Ld_mm < 50.0f) Ld_mm = 50.0f;

    float alpha_rad = alpha_deg * DEG_TO_RAD;

    // Pure Pursuit: delta = atan2(2 * L * sin(alpha), Ld)
    float steering_rad = atan2(2.0f * WHEELBASE_MM * sin(alpha_rad), Ld_mm);

    float steering_deg = steering_rad * RAD_TO_DEG;

    // Clamp to the mechanical limit.
    steering_deg = constrain(steering_deg, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    return steering_deg;
}
