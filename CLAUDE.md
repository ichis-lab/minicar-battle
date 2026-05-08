# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working with this repository.

## Project overview

An autonomous-driving project for a competition mini car based on the Tamiya TT-02 chassis.

**Hardware control system** (`seven/`): the autonomous driving stack on Arduino Nano R4 with seven VL53L1X ToF sensors.

### Race goals
- **Time trial**: lap time below 9 seconds.
- **Endurance**: continuous racing for 6 minutes (no wall hits, checkpoints in order).

### Race result
- **Final**: 3 laps in 17.5s (1st place).

## Repository structure

```
minicar-battle/
├── seven/                           # Arduino autonomous driving stack
│   ├── seven.ino                   # main sketch (entry point)
│   ├── Config.h                    # all tunables and constants
│   ├── SensorReader.cpp/h          # VL53L1X reads
│   ├── GapFinder.cpp/h             # target angle from farthest sensor + neighbors
│   ├── SteeringController.cpp/h    # steering via Pure Pursuit
│   ├── Actuator.cpp/h              # servo/ESC PWM output
│   └── Logger.h                    # debug logger (header-only)
├── CLAUDE.md                       # this file
├── README.md                       # detailed project documentation
└── LICENSE                         # MIT
```

---

## System architecture

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

### Hardware requirements

- **MCU**: Arduino Nano R4
- **Sensors**: VL53L1X ToF x 7
- **I2C multiplexer**: TCA9548A (address 0x70)
- **Actuators**: hobby servo (steering) and ESC (drive)

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

| Sensor   | Channel | Angle  | Role          |
|----------|---------|--------|---------------|
| Sensor 0 | CH0     | -60deg | left side     |
| Sensor 1 | CH1     | -30deg | left front    |
| Sensor 2 | CH2     | -15deg | left forward  |
| Sensor 3 | CH3     | 0deg   | front         |
| Sensor 4 | CH4     | +15deg | right forward |
| Sensor 5 | CH5     | +30deg | right front   |
| Sensor 6 | CH6     | +60deg | right side    |

---

## Control algorithm: Follow-the-Gap + Pure Pursuit

### Farthest-sensor + neighbors method (GapFinder)

1. Pick the farthest valid sensor out of the 7 (with hysteresis).
2. Take its left/right neighbors when available (only one side at the array ends).
3. Distance-weight the farthest sensor and its neighbors to get target_angle.

### Pure Pursuit (SteeringController)

```
steering_angle = atan2(2 * L * sin(alpha), Ld)
```

- **L**: wheelbase (mm). MF-01X = 210mm.
- **alpha**: angle to the target point (rad). Comes from GapFinder.target_angle.
- **Ld**: lookahead distance (mm). **Front sensor (S3) distance minus body length (1200mm).**

### Control flow

```
1. read sensors (7 sensors per cycle)
      v
2. emergency-stop check (front < 400mm)
      v
3. pick the farthest sensor and compute target angle (GapFinder)
      v
4. Ld = front sensor distance - body length (SteeringController)
      v
5. derive steering angle via Pure Pursuit
      v
6. PWM out (servo + ESC cruise)
```

---

## Running the system

### 1. Install libraries

In the Arduino IDE, install:
- **VL53L1X** (Pololu)
- **Servo** (Arduino built-in)

### 2. Configure

Pick a `RUN_MODE` in `Config.h`:
```cpp
#define MODE_DEBUG 0       // debug only (no PWM, serial enabled)
#define MODE_PRODUCTION 1  // race run (PWM enabled, serial disabled)
#define MODE_DEBUG_RUN 2   // debug run (PWM and serial enabled)

#define RUN_MODE MODE_PRODUCTION  // pick the run mode here
```

### 3. Flash

```bash
# With Arduino CLI
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi seven
arduino-cli upload -p /dev/cu.usbmodem* --fqbn arduino:renesas_uno:unor4wifi seven
```

---

## Key configuration parameters

In `seven/Config.h`:

```cpp
// Loop timing
const unsigned long MEASUREMENT_INTERVAL = 40;  // main loop period (ms)

// Pure Pursuit
const float WHEELBASE_MM = 210.0;            // wheelbase (mm), MF-01X
const float LOOKAHEAD_OFFSET_MM = 1200.0;    // body length (mm), sensor mount to rear axle

// Safety
const uint16_t EMERGENCY_FRONT_THRESHOLD = 400;  // front emergency threshold (mm)

// Cruise speed
const uint16_t SPEED_US = 1680;  // cruise pulse (us)

// Servo (us)
const uint16_t SERVO_CENTER = 1415;  // center
const uint16_t SERVO_MIN = 1115;     // min pulse (full right)
const uint16_t SERVO_MAX = 1715;     // max pulse (full left)
```

---

## Debug output format

Sample serial output when `RUN_MODE=MODE_DEBUG_RUN`:

```
S0:1234 | S1:567 | S2:890 | S3:456 | S4:789 | S5:321 | S6:654 | T:15.0° Ld:600 St:13.5 RR | T:28000us(S:25000us)
```

| Field  | Description                       |
|--------|-----------------------------------|
| S0-S6  | Distance per sensor (mm)          |
| T      | Target angle (deg)                |
| Ld     | Lookahead distance (mm)           |
| St     | Steering angle (deg)              |
| L/R    | Steering-direction indicator      |
| T      | Loop time (us)                    |

---

## Safety features

1. **Emergency stop**: stops automatically when the front sensor reads under 400mm.

---

## Common issues

**Q: a sensor fails to initialize**
-> check the I2C wiring, the TCA9548A address (0x70), and VL53L1X power.

**Q: the servo or ESC does not move**
-> confirm `RUN_MODE` is `MODE_PRODUCTION` or `MODE_DEBUG_RUN`.
-> run ESC calibration.

**Q: steering goes the wrong way**
-> swap `SERVO_MIN` and `SERVO_MAX` in `Config.h`.

---

## Collaboration guidelines

These notes apply when iterating on this project.

### 1. Check the impact scenes before changing parameters

Before proposing or implementing a parameter or algorithm change, walk through how it affects the three driving scenes below:

| Scene          | What to check                              |
|----------------|--------------------------------------------|
| **Straight**   | Does it add yaw oscillation?               |
| **Soft turn**  | Does it still corner smoothly?             |
| **Hard turn (S-curve)** | Does it understeer or push wide? |

Sample prompts:
- "This change affects straight-line behavior too. Are we sure it will not add oscillation?"
- "Should we also walk through the S-curve case before testing?"

### 2. Record why a setting was rejected

When a parameter or approach is tried and rejected, capture the reason as a comment in `Config.h`:

```cpp
// History:
// LD_OFFSET = 1200: rejected (more straight-line oscillation, 2025-01-25)
// LD_OFFSET = 1050: adopted (3 laps in 18.53s, better S-curves)
```

Sample prompts:
- "Want to leave a note in `Config.h` with the result and why?"
- "Worth recording the rejection reason for future reference."

### 3. Make the evaluation axes explicit

When comparing approaches, surface the axes you are weighing:

| Axis              | What it covers                                |
|-------------------|-----------------------------------------------|
| **Lap time**      | Effect on lap time (seconds)                  |
| **Stability**     | Risk of yaw oscillation or wall hits          |
| **Cost / risk**   | Implementation effort and rollback risk       |

Sample prompts:
- "Are we optimizing for lap time or for stability here?"
- "Let's lay out the axes: time / stability / cost."

### 4. Classify the change level

Be explicit about the size of the proposed change:

| Level | Scope                       | Process                                |
|-------|-----------------------------|----------------------------------------|
| **L1**| Single-parameter tweak      | Test directly                          |
| **L2**| Algorithm tweak             | Verify impact across modules first     |
| **L3**| Architectural change        | Cost-benefit analysis first            |

Sample prompts:
- "This is L2 (algorithm tweak), so let's check both GapFinder and SteeringController first."
- "L3 territory. Want to do a cost-benefit pass before starting?"

---

## Branch strategy

- `main`: main branch (production).
- `feat/*`: feature branches.

---

## Language

Code, comments, and documentation are written in English. Conversational replies in chat may be in Japanese.
