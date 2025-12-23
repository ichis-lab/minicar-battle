/*
 * SteeringController.h
 *
 * シンプル状態ベース制御
 */

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "SensorReader.h"

enum ControlMode {
    MODE_STRAIGHT,
    MODE_CORNER,
    MODE_EMERGENCY,
    MODE_SIDE_AVOID
};

class SteeringController {
private:
    ControlMode _currentMode;
    float _lastSteering;
    float _wallAngle;

public:
    SteeringController();
    void begin();
    float calculate(const SensorData* sensors);

    ControlMode getCurrentMode() const { return _currentMode; }
    const char* getModeName() const;
    float getWallAngle() const { return _wallAngle; }
};

#endif
