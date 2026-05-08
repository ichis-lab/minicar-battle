# Seven - autonomous driver based on the farthest-sensor + neighbors method

An autonomous driving control system for the Arduino Nano R4, built on seven VL53L1X ToF sensors and the farthest-sensor + neighbors method.

**Race result: 3 laps in 17.5s in the final (1st place).**

---

## System overview

### Architecture

```
+------------------------------------------------------------------+
|                       Arduino Nano R4                            |
|                                                                  |
|  +--------------+    +--------------+    +-------------------+   |
|  | SensorReader |--->|  GapFinder   |--->| SteeringController|   |
|  |  (I2C read)  |    | (target deg) |    |  (Pure Pursuit)   |   |
|  +--------------+    +--------------+    +---------+---------+   |
|                                                    |             |
|  +--------------+      +---------------------------v--------+    |
|  |  TCA9548A    |      |            Actuator                |    |
|  |  multiplexer |      |     (PWM out + cruise control)     |    |
|  +--------------+      +-------------------------------------+   |
+-------+--------------------------------------+-------------------+
        |                                      |
   +----+----+                            +----+-----+
   | VL53L1X | x 7                        | servo/ESC|
   +---------+                            +----------+
```

### Sensor layout

```
                   front (Sensor 3: 0deg)
                         |
              -15deg     |     +15deg
               (S2)      |      (S4)
         -30deg   \      |      /   +30deg
          (S1)     \     |     /     (S5)
    -60deg          \    |    /          +60deg
  (Sensor 0)             |             (Sensor 6)
     left                |                   right
```

| Sensor   | Channel | Angle  | Role         |
|----------|---------|--------|--------------|
| Sensor 0 | CH0     | -60deg | left side    |
| Sensor 1 | CH1     | -30deg | left front   |
| Sensor 2 | CH2     | -15deg | left forward |
| Sensor 3 | CH3     | 0deg   | front        |
| Sensor 4 | CH4     | +15deg | right forward|
| Sensor 5 | CH5     | +30deg | right front  |
| Sensor 6 | CH6     | +60deg | right side   |

---

## File layout

```
seven/
├── seven.ino                   # main sketch (entry point)
├── Config.h                    # all tunables and constants
├── SensorReader.cpp/h          # VL53L1X reads via TCA9548A
├── GapFinder.cpp/h             # target angle from farthest sensor + neighbors
├── SteeringController.cpp/h    # steering angle via Pure Pursuit
├── Actuator.cpp/h              # servo/ESC PWM output (cruise speed)
└── Logger.h                    # header-only debug logger
```

---

## Farthest-sensor + neighbors method

### Idea

Pick the farthest valid sensor and combine it with its neighbors to compute a target angle by distance weighting.

### Algorithm

1. Pick the farthest valid sensor out of the 7.
2. Take its left/right neighbors when available (only one side at the array ends).
3. Compute the target angle as a distance-weighted blend of those readings.

### Pipeline

```
1. read sensors
      v
2. pick the farthest sensor (with hysteresis)
      v
3. compute target angle from the farthest + neighbors
      v
4. Ld = front sensor distance - body length
      v
5. derive steering angle via Pure Pursuit
```

### Why this method

- **Stable on straights**: blending three readings smooths jitter.
- **Tight corners**: no absolute distance threshold, so it still works in narrow sections.
- **Simple**: no complex gap scoring or segmentation.

### Edge sensors

- Sensor 0 (-60deg) is farthest -> use sensors 0 and 1.
- Sensor 6 (+60deg) is farthest -> use sensors 5 and 6.

---

## Pure Pursuit

### Idea

Pure Pursuit traces an arc from the rear axle to a target point. The target heading and lookahead distance from GapFinder feed straight into Pure Pursuit.

### Formula

```
steering_angle = atan2(2 * L * sin(alpha), Ld)
```

- **L**: wheelbase (mm). MF-01X = 210mm.
- **alpha**: angle to the target point (rad). Comes from GapFinder.target_angle.
- **Ld**: lookahead distance (mm). Front sensor (S3) distance minus body length (1200mm), with a minimum of 50mm.

### Properties

- **Distance-aware**: the same angle yields a tighter turn at short Ld and a softer turn at long Ld.
- **Geometric**: based on the vehicle's kinematics, not heuristics.
- **Stateless**: no history to drift, so it reacts immediately to changes.

### Tunables

| Name                  | Value | Effect                                            |
|-----------------------|-------|---------------------------------------------------|
| WHEELBASE_MM          | 210   | Wheelbase. Larger -> softer turns.                |
| LOOKAHEAD_OFFSET_MM   | 1200  | Ld offset. Tuned manually.                        |

---

## Config.h reference

### Run mode

| Constant         | Value | Description                              |
|------------------|-------|------------------------------------------|
| `MODE_DEBUG`     | 0     | Debug only (no PWM, serial enabled)      |
| `MODE_PRODUCTION`| 1     | Race run (PWM enabled, serial disabled)  |
| `MODE_DEBUG_RUN` | 2     | Debug run (PWM and serial enabled)       |
| `RUN_MODE`       | -     | Pick one of the above                    |

### Hardware

| Name                 | Type     | Value                          | Description                       |
|----------------------|----------|--------------------------------|-----------------------------------|
| `TCA9548A_ADDR`      | uint8_t  | 0x70                           | I2C multiplexer address           |
| `NUM_SENSORS`        | uint8_t  | 7                              | Sensor count                      |
| `SENSOR_ANGLES`      | float[]  | {-60,-30,-15,0,15,30,60}       | Mounting angle per sensor (deg)   |
| `FRONT_SENSOR_INDEX` | uint8_t  | 3                              | Index of the front sensor (0deg)  |
| `SERVO_PIN`          | uint8_t  | 9                              | Steering servo pin                |
| `ESC_PIN`            | uint8_t  | 10                             | ESC pin                           |

### Sensor parameters

| Name                       | Type     | Value | Description                              |
|----------------------------|----------|-------|------------------------------------------|
| `MIN_VALID_DISTANCE`       | uint16_t | 50    | Minimum valid range (mm)                 |
| `RELIABLE_RANGE`           | uint16_t | 4000  | Trusted measurement range (mm)           |
| `L1X_TIMING_BUDGET_US`     | uint32_t | 33000 | VL53L1X measurement budget (us)          |
| `L1X_INTER_MEASUREMENT_MS` | uint32_t | 40    | VL53L1X inter-measurement period (ms)    |

### Loop timing

| Name                   | Type           | Value | Description           |
|------------------------|----------------|-------|-----------------------|
| `MEASUREMENT_INTERVAL` | unsigned long  | 40    | Main loop period (ms) |

### Steering

| Name                 | Type  | Value | Description                |
|----------------------|-------|-------|----------------------------|
| `MAX_STEERING_ANGLE` | float | 30.0  | Max steering angle (deg)   |

### Pure Pursuit

| Name                  | Type  | Value  | Description                                         |
|-----------------------|-------|--------|-----------------------------------------------------|
| `WHEELBASE_MM`        | float | 210.0  | Wheelbase (mm)                                      |
| `LOOKAHEAD_OFFSET_MM` | float | 1200.0 | Ld offset (mm). Tuned manually.                     |

### Gap detection

| Name                  | Type  | Value | Description                                            |
|-----------------------|-------|-------|--------------------------------------------------------|
| `FARTHEST_HYSTERESIS` | float | 100.0 | Hysteresis for switching the farthest sensor (mm)      |

Hysteresis behavior:
- The farthest sensor only changes when a new candidate beats the current one by at least 100mm.
- Prevents flapping when readings hover around each other.

### Safety

| Name                        | Type     | Value | Description                          |
|-----------------------------|----------|-------|--------------------------------------|
| `EMERGENCY_FRONT_THRESHOLD` | uint16_t | 400   | Front emergency-stop threshold (mm)  |

### Servo (us)

| Name           | Type     | Value | Description           |
|----------------|----------|-------|-----------------------|
| `SERVO_CENTER` | uint16_t | 1415  | Center                |
| `SERVO_MIN`    | uint16_t | 1115  | Min pulse (full right)|
| `SERVO_MAX`    | uint16_t | 1715  | Max pulse (full left) |

### ESC (us)

| Name          | Type     | Value | Description |
|---------------|----------|-------|-------------|
| `ESC_STOP_US` | uint16_t | 1500  | Stop        |
| `ESC_MIN_US`  | uint16_t | 1000  | Min pulse   |
| `ESC_MAX_US`  | uint16_t | 2000  | Max pulse   |

### Cruise speed

| Name       | Type     | Value | Description                    |
|------------|----------|-------|--------------------------------|
| `SPEED_US` | uint16_t | 1680  | Cruise pulse width (us)        |

---

## Hardware

- **MCU**: Arduino Nano R4
- **Sensors**: VL53L1X ToF x 7
- **I2C multiplexer**: TCA9548A
- **Steering**: hobby servo
- **Drive**: ESC + brushless motor

### Wiring

| From            | To                       |
|-----------------|--------------------------|
| Arduino SDA     | TCA9548A SDA             |
| Arduino SCL     | TCA9548A SCL             |
| TCA9548A CH0-6  | VL53L1X x 7              |
| Arduino pin 9   | Steering servo signal    |
| Arduino pin 10  | ESC signal               |

---

## Usage

### 1. Install libraries

In the Arduino IDE, install:
- **VL53L1X** (Pololu)
- **Servo** (Arduino built-in)

### 2. Configure

Set `RUN_MODE` in `Config.h`:
- `MODE_DEBUG`: debug mode (PWM disabled, serial enabled)
- `MODE_PRODUCTION`: race mode (PWM enabled, serial disabled)
- `MODE_DEBUG_RUN`: debug run (PWM and serial enabled)

### 3. Flash

Upload the sketch to the Arduino Nano R4.

---

## Debug output

Sample serial output when `RUN_MODE=MODE_DEBUG_RUN`:

```
S0:1234 | S1:567 | S2:890 | S3:456 | S4:789 | S5:321 | S6:654 | T:15.0° Ld:600 St:13.5 RR | T:28000us(S:25000us)
```

| Field  | Description                          |
|--------|--------------------------------------|
| S0-S6  | Distance per sensor (mm)             |
| T      | Target angle (deg)                   |
| Ld     | Lookahead distance (mm)              |
| St     | Steering angle (deg)                 |
| L/R    | Steering-direction indicator         |
| T      | Loop time (us)                       |

---

## Safety

1. **Emergency stop**: stops automatically when the front sensor reads under 400mm.
