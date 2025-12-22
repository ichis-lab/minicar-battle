/*
 * ArduinoMock.cpp - millis()用グローバル変数の実体
 */

#include "Arduino.h"

namespace ArduinoMock {
    unsigned long mock_millis = 0;
}
