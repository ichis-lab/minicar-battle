/*
  シンプルなVL53L0Xセンサー読み取りテスト（ループ版）
  TCA9548Aマルチプレクサ経由で複数のセンサーからデータを取得

  接続:
  - TCA9548A I2Cマルチプレクサ (アドレス: 0x70)
  - VL53L0Xセンサー → マルチプレクサのチャンネル0, 1に接続
*/

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// TCA9548Aマルチプレクサのアドレス
#define TCA9548A_ADDR 0x70

// センサーの数（ここを変更するだけで増減可能）
#define NUM_SENSORS 5

// 各センサーが接続されているチャンネル
const uint8_t sensorChannels[NUM_SENSORS] = {0, 1, 2, 3, 4};

// VL53L0Xセンサーオブジェクトの配列
Adafruit_VL53L0X sensors[NUM_SENSORS];

// 測定データの配列
VL53L0X_RangingMeasurementData_t measurements[NUM_SENSORS];

/**
 * TCA9548Aマルチプレクサのチャンネルを選択
 */
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.print("VL53L0X Test with ");
  Serial.print(NUM_SENSORS);
  Serial.println(" Sensors");
  Serial.println("==========================================");

  // 全センサーを初期化
  for (int i = 0; i < NUM_SENSORS; ++i) {
    tcaSelect(sensorChannels[i]);
    delay(1000);

    Serial.print("Initializing sensor ");
    Serial.print(i);
    Serial.print(" on channel ");
    Serial.print(sensorChannels[i]);
    Serial.print("...");

    if (!sensors[i].begin()) {
      Serial.println("FAILED!");
      Serial.print("Check sensor ");
      Serial.print(i);
      Serial.println(" connections!");
      while (1);
    }

    Serial.println("OK!");
  }

  Serial.println();
}

void loop() {
  // 全センサーから測定
  for (int i = 0; i < NUM_SENSORS; ++i) {
    tcaSelect(sensorChannels[i]);
    sensors[i].rangingTest(&measurements[i], false);
  }

  // 結果表示
  for (int i = 0; i < NUM_SENSORS; ++i) {
    Serial.print("Ch");
    Serial.print(sensorChannels[i]);
    Serial.print(": ");

    if (measurements[i].RangeStatus != 4) {
      Serial.print(measurements[i].RangeMilliMeter);
      Serial.print(" mm");
    } else {
      Serial.print("Out of range");
    }

    // 最後のセンサー以外は区切り文字を表示
    if (i < NUM_SENSORS - 1) {
      Serial.print("  |  ");
    }
  }

  Serial.println();

  delay(100);
}
