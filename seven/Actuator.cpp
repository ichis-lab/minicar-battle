/*
 * Actuator.cpp
 *
 * PWM output for the steering servo and the ESC.
 */

#include "Actuator.h"

#include "Logger.h"

Actuator::Actuator() {
    // Nothing to do here.
}

void Actuator::begin() {
#if ENABLE_PWM
    _steeringServo.attach(SERVO_PIN);
    _escController.attach(ESC_PIN);

    // Drive both outputs to a safe initial value.
    _steeringServo.writeMicroseconds(SERVO_CENTER);
    _escController.writeMicroseconds(ESC_STOP_US);

    Logger::println("Actuators initialized:");
    Logger::print("  Servo: Pin ");
    Logger::println(SERVO_PIN);
    Logger::print("  ESC: Pin ");
    Logger::println(ESC_PIN);
#else
    Logger::println("DEBUG MODE: Actuators disabled (no PWM output)");
#endif
}

void Actuator::setSteering(float angle_degrees) {
    // Map the angle (in tenths of a degree) to a pulse width.
    int pulse_us =
        map((int)(angle_degrees * 10),
            (int)(-MAX_STEERING_ANGLE * 10),
            (int)(MAX_STEERING_ANGLE * 10),
            SERVO_MAX, SERVO_MIN);

    // Clamp to the configured pulse-width limits.
    if (pulse_us < SERVO_MIN) pulse_us = SERVO_MIN;
    if (pulse_us > SERVO_MAX) pulse_us = SERVO_MAX;

#if ENABLE_PWM
    _steeringServo.writeMicroseconds(pulse_us);
#endif

    Logger::printActuator("Servo", pulse_us);
}

void Actuator::setSpeed(uint16_t pulse_us) {
    // Clamp to the configured pulse-width limits.
    if (pulse_us < ESC_MIN_US) pulse_us = ESC_MIN_US;
    if (pulse_us > ESC_MAX_US) pulse_us = ESC_MAX_US;

#if ENABLE_PWM
    _escController.writeMicroseconds(pulse_us);
#endif

    Logger::printActuator("ESC", pulse_us);
}

void Actuator::stop() { setSpeed(ESC_STOP_US); }
