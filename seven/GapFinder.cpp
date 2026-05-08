/*
 * GapFinder.cpp
 *
 * Algorithm:
 * 1. Pick the farthest valid sensor (with hysteresis to avoid flapping).
 * 2. Look at its left/right neighbors when available.
 * 3. With both neighbors: weight each neighbor by the area of the triangle
 *    it forms with the farthest sensor, and take the area-weighted angle.
 *    With one neighbor (or none): fall back to a distance-weighted angle.
 *
 * Triangle areas (the 1/2 cancels in the ratio, so it is omitted):
 *   area_left  = d_left  * d_farthest * sin(angle gap to left)
 *   area_right = d_right * d_farthest * sin(angle gap to right)
 *   target = (angle_left * area_left + angle_right * area_right) /
 *            (area_left + area_right)
 *
 * Note: Ld (the lookahead distance) is computed in SteeringController from
 * the front sensor reading, not here.
 */

#include "GapFinder.h"

#include <math.h>

GapFinder::GapFinder() : _lastFarthestIdx(FRONT_SENSOR_INDEX) {}

// ============================================================================
// Private
// ============================================================================

int GapFinder::_findFarthestSensor(const SensorData* data,
                                  float& outDistance) const {
    int farthest_idx = -1;
    float farthest_dist = 0.0f;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        if (data[i].valid && data[i].distance > farthest_dist) {
            farthest_dist = data[i].distance;
            farthest_idx = i;
        }
    }

    outDistance = farthest_dist;
    return farthest_idx;
}

int GapFinder::_applyHysteresis(const SensorData* data, int candidateIdx,
                               float candidateDist, float& outDistance) {
    int result_idx = candidateIdx;
    float result_dist = candidateDist;

    // Keep the previous farthest sensor if the new candidate is not enough farther.
    if (_lastFarthestIdx >= 0 && _lastFarthestIdx < NUM_SENSORS &&
        data[_lastFarthestIdx].valid && candidateIdx != _lastFarthestIdx) {
        float diff = candidateDist - data[_lastFarthestIdx].distance;
        if (diff < FARTHEST_HYSTERESIS) {
            result_idx = _lastFarthestIdx;
            result_dist = data[_lastFarthestIdx].distance;
        }
    }

    // Update state for the next call.
    _lastFarthestIdx = result_idx;
    outDistance = result_dist;
    return result_idx;
}

float GapFinder::_calculateTargetAngle(const SensorData* data, int farthestIdx,
                                      float farthestDist) const {
    int left_idx = (farthestIdx > 0) ? farthestIdx - 1 : -1;
    int right_idx = (farthestIdx < NUM_SENSORS - 1) ? farthestIdx + 1 : -1;

    bool has_left = (left_idx >= 0 && data[left_idx].valid);
    bool has_right = (right_idx >= 0 && data[right_idx].valid);

    if (has_left && has_right) {
        // Both neighbors valid: triangle-area weighting.
        float angle_diff_left =
            fabs(SENSOR_ANGLES[farthestIdx] - SENSOR_ANGLES[left_idx]);
        float angle_diff_right =
            fabs(SENSOR_ANGLES[right_idx] - SENSOR_ANGLES[farthestIdx]);

        // Use the precomputed sin values (15 deg or 20 deg).
        float sin_left = (angle_diff_left < 17.5f) ? SIN_15_DEG : SIN_20_DEG;
        float sin_right = (angle_diff_right < 17.5f) ? SIN_15_DEG : SIN_20_DEG;

        float area_left = data[left_idx].distance * farthestDist * sin_left;
        float area_right = farthestDist * data[right_idx].distance * sin_right;

        float total_area = area_left + area_right;
        if (total_area < 0.001f) {
            // Avoid divide-by-zero; fall back to the farthest sensor's angle.
            return SENSOR_ANGLES[farthestIdx];
        }
        return (SENSOR_ANGLES[left_idx] * area_left +
                SENSOR_ANGLES[right_idx] * area_right) / total_area;
    } else {
        // One side or zero neighbors: distance-weighted average.
        float weighted_sum = SENSOR_ANGLES[farthestIdx] * farthestDist;
        float weight_total = farthestDist;

        if (has_left) {
            weighted_sum += SENSOR_ANGLES[left_idx] * data[left_idx].distance;
            weight_total += data[left_idx].distance;
        }
        if (has_right) {
            weighted_sum += SENSOR_ANGLES[right_idx] * data[right_idx].distance;
            weight_total += data[right_idx].distance;
        }

        if (weight_total < 0.001f) {
            // Avoid divide-by-zero; fall back to the farthest sensor's angle.
            return SENSOR_ANGLES[farthestIdx];
        }
        return weighted_sum / weight_total;
    }
}

// ============================================================================
// Public
// ============================================================================

GapResult GapFinder::find(const SensorData* data) {
    GapResult result = {0.0f};

    // Step 1: pick the farthest sensor.
    float farthest_dist;
    int farthest_idx = _findFarthestSensor(data, farthest_dist);

    // No valid sensor -> drive straight.
    if (farthest_idx < 0) {
        return result;
    }

    // Step 2: hysteresis.
    farthest_idx = _applyHysteresis(data, farthest_idx, farthest_dist,
                                   farthest_dist);

    // Step 3: target angle.
    result.target_angle = _calculateTargetAngle(data, farthest_idx,
                                               farthest_dist);

    return result;
}
