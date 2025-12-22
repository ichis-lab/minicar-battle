/*
 * Logger.h - テスト用スタブ
 *
 * テスト時は出力を無効化
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <cstdint>

class Logger {
public:
    static void begin(unsigned long baud = 9600) { (void)baud; }

    template<typename T>
    static void print(const T& value) { (void)value; }

    template<typename T>
    static void println(const T& value) { (void)value; }

    static void println() {}

    static void printSensorData(uint8_t channel, uint16_t distance, bool valid) {
        (void)channel; (void)distance; (void)valid;
    }

    static void printWallStatus(bool left_valid, bool right_valid) {
        (void)left_valid; (void)right_valid;
    }

    static void printWallDistances(bool left_valid, float left_dist, bool right_valid, float right_dist) {
        (void)left_valid; (void)left_dist; (void)right_valid; (void)right_dist;
    }

    static void printSteering(float angle) { (void)angle; }

    static void printActuator(const char* name, uint16_t pulse_us) {
        (void)name; (void)pulse_us;
    }
};

#endif // LOGGER_H
