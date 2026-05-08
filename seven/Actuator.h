/*
 * Actuator.h
 *
 * PWM control for the steering servo and the ESC.
 * Output is gated on RUN_MODE (see Config.h).
 */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <Servo.h>

#include "Config.h"

class Actuator {
   private:
    Servo _steeringServo;
    Servo _escController;

   public:
    Actuator();

    // Attach the servo and ESC and set them to safe defaults.
    void begin();

    // Set the steering angle in degrees.
    void setSteering(float angle_degrees);

    // Set the ESC throttle as a raw pulse width (us).
    void setSpeed(uint16_t speed_pulse_us);

    // Cut throttle.
    void stop();
};

#endif  // ACTUATOR_H
