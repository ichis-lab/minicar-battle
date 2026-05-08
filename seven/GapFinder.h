/*
 * GapFinder.h
 *
 * Finds the steering target by combining the farthest sensor with its neighbors.
 * The target angle is computed by triangle-area weighting when both neighbors
 * are valid, and falls back to distance weighting otherwise. See GapFinder.cpp
 * for details.
 *
 * State:
 * - _lastFarthestIdx: the previous farthest-sensor index. Used by the
 *   hysteresis check so that a new sensor only wins when it is at least
 *   FARTHEST_HYSTERESIS (mm) farther than the previous one.
 */

#ifndef GAP_FINDER_H
#define GAP_FINDER_H

#include <Arduino.h>

#include "Config.h"
#include "SensorReader.h"

// Output of GapFinder::find.
struct GapResult {
    float target_angle;  // target heading (deg); positive = right, negative = left
};

class GapFinder {
   private:
    int _lastFarthestIdx;  // previous farthest-sensor index (for hysteresis)

    // Pick the farthest sensor before applying hysteresis.
    int _findFarthestSensor(const SensorData* data, float& outDistance) const;

    // Apply hysteresis and return the final farthest-sensor index.
    int _applyHysteresis(const SensorData* data, int candidateIdx,
                         float candidateDist, float& outDistance);

    // Compute the target angle from the farthest sensor and its neighbors.
    float _calculateTargetAngle(const SensorData* data, int farthestIdx,
                                float farthestDist) const;

   public:
    GapFinder();

    // Main entry point: turn sensor data into a target angle.
    GapResult find(const SensorData* sensorData);
};

#endif  // GAP_FINDER_H
