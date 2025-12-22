/*
 * SensorReader.h - テスト用（構造体定義のみ）
 *
 * SensorData構造体をWallDetectorで使用するため
 */

#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <cstdint>
#include "Config.h"

// センサーデータ構造体
struct SensorData {
    uint16_t distance;  // 測定距離（mm）
    bool valid;         // 測定値の有効性
    uint8_t status;     // VL53L0Xステータスコード
};

#endif // SENSOR_READER_H
