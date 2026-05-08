/*
 * SteeringController.h
 *
 * Steering control via Pure Pursuit on top of the Follow-the-Gap target.
 */

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>

#include "Config.h"
#include "GapFinder.h"

class SteeringController {
   public:
    SteeringController();

    // Init (no-op; included for symmetry with the other modules).
    void begin();

    // Compute the steering angle (deg) using Pure Pursuit.
    float calculate(const GapResult& gap, const SensorData* sensorData);
};

#endif  // STEERING_CONTROLLER_H
