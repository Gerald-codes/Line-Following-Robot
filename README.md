# LINE FOLLOWING ROBOT - TEAM PROJECT

## 🎯 Project Overview

A professional line-following robot with:
- ✅ PID line following (works on curves!)
- ✅ IMU-based 90° turns
- ✅ Obstacle detection and avoidance
- ✅ Barcode command recognition
- ✅ Clean state machine architecture
- ✅ Parallel development structure

## 📁 Project Structure

```
robotcar_team/
│
├── README.md                    # This file
├── TEAM_TASK_ASSIGNMENT.md      # Task breakdown for team
│
├── include/                     # All header files
│   ├── config.h                # ⚙️ Configuration (TUNE HERE!)
│   ├── pin_definitions.h       # GPIO pin assignments
│   ├── common.h                # Common types & utilities
│   ├── state_machine.h         # State machine interface
│   └── [subsystem headers...]  # To be created by team
│
└── src/
    ├── main.c                   # 🚀 Main program (DONE)
    │
    ├── state_machine/
    │   └── state_machine.c     # 🧠 State machine logic (DONE)
    │
    ├── drivers/                # Hardware drivers (TEAM implements)
    │   ├── motor.c             # PWM motor control
    │   ├── encoder.c           # Interrupt-based encoders
    │   ├── imu.c               # Magnetometer + accel
    │   ├── ir_sensor.c         # Line + barcode sensors
    │   ├── ultrasonic.c        # Distance measurement
    │   └── servo.c             # Servo control
    │
    ├── subsystems/             # Mid-level logic (TEAM implements)
    │   ├── line_follower.c     # Line following with PID
    │   ├── navigation.c        # IMU-based turns
    │   ├── obstacle_avoidance.c # Obstacle handling
    │   └── barcode_decoder.c   # Code 39 decoder
    │
    └── control/
        └── pid.c               # Generic PID controller
```

## 🚀 Quick Start

### 1. Understand the Architecture

**Main Loop (50Hz):**
```
while (running):
    1. Read all sensors
    2. Update state machine (decide what to do)
    3. Execute state actions (control motors)
    4. Send telemetry (optional)
    5. Maintain 50Hz timing
```

**State Machine Flow:**
```
IDLE → CALIBRATING → LINE_FOLLOWING
                          ↓
            ┌─────────────┴─────────────┐
            ↓                           ↓
      BARCODE_DETECTED          OBSTACLE_DETECTED
            ↓                           ↓
      TURNING_LEFT/RIGHT          OBSTACLE_SCANNING
            ↓                           ↓
      LINE_FOLLOWING ←────────── OBSTACLE_AVOIDING
                                        ↓
                                  SEARCHING_LINE
```

### 2. Review Core Files

**Start here:**
1. Read `TEAM_TASK_ASSIGNMENT.md` - Understand your task
2. Read `include/config.h` - See all tunable parameters
3. Read `src/main.c` - Understand system flow
4. Read `src/state_machine/state_machine.c` - See state logic

### 3. Configure Hardware

**Edit `include/pin_definitions.h`:**
```c
// Update these to match your wiring!
#define M1A     8   // Left motor
#define M1B     9
#define M2A     10  // Right motor
#define M2B     11
// ... etc
```

**Edit `include/config.h`:**
```c
// Your calibrated values (ALREADY SET!)
#define WHEEL_DIAMETER_MM       65.0f
#define LEFT_PULSES_PER_REV     50.163f
#define RIGHT_PULSES_PER_REV    47.618f

// TUNE THESE for line following:
#define LINE_PID_KP             35.0f
#define LINE_PID_KI             0.05f
#define LINE_PID_KD             8.0f
```

## 👥 Team Development

### Phase 1: Implement Drivers (Week 1)

**Each team member implements ONE driver:**

```bash
# Example: Member working on motor control
cd robotcar_team/src/drivers
# Create motor.c based on interface in include/motor.h
# Test independently
```

**Drivers to implement:**
1. `motor.c` - PWM control
2. `encoder.c` - Interrupt counting
3. `imu.c` - Compass heading (you already have this!)
4. `ir_sensor.c` - Line + barcode sensors
5. `ultrasonic.c` + `servo.c` - Distance + scanning

**Testing:** Each driver should work standalone before integration

### Phase 2: Implement Subsystems (Week 2)

**Build mid-level logic:**

1. `pid.c` - Generic PID algorithm
2. `line_follower.c` - PID line following + adaptive speed
3. `navigation.c` - IMU-based turns
4. `obstacle_avoidance.c` - Servo scan + maneuvers
5. `barcode_decoder.c` - Code 39 decoding

**Critical:** Tune `line_follower.c` for curves!

### Phase 3: Integration & Testing (Week 3-4)

1. Integrate all subsystems
2. Test state transitions
3. Run full course
4. Tune PID gains
5. Fix edge cases
6. Final demo prep

## 🔧 Development Workflow

### Option A: Copy Your Existing Code

You already have working code! Just adapt it:

```bash
# Copy your working IMU code
cp /your/old/project/imu.c src/drivers/
cp /your/old/project/imu.h include/

# Adapt the interface to match common.h types
# - IMU struct is already defined in common.h
# - Functions match the interface
```

### Option B: Start Fresh

Use the interfaces provided and implement from scratch.

### Recommended: Hybrid Approach

1. Copy working drivers (motor, encoder, IMU)
2. Adapt interfaces to match
3. Implement new subsystems (line_follower, etc.)
4. Integrate with state machine

## 📊 What's Already Done

### ✅ Core Framework (100%)
- [x] Main loop structure
- [x] State machine with 13 states
- [x] Sensor data structures
- [x] State context tracking
- [x] Configuration system
- [x] Clean interfaces

### ✅ Documentation (100%)
- [x] Task assignments
- [x] Interface definitions
- [x] Development phases
- [x] Testing checklist

### ⚠️ To Be Implemented (0%)
- [ ] All drivers (motor, encoder, IMU, sensors)
- [ ] All subsystems (line follower, navigation, etc.)
- [ ] PID controller
- [ ] Integration testing

## 🎓 Key Concepts

### State Machine Pattern

**Separation of concerns:**
- `state_machine_update()` = **BRAIN** (decides what to do)
- `state_machine_execute()` = **MUSCLES** (does the action)

This makes code easy to understand and debug!

### Adaptive Line Following

**For curves, the robot needs to:**
1. Detect it's on a curve (large error)
2. Slow down
3. Increase PID aggressiveness
4. Speed up on straight sections

```c
// In line_follower.c
if (abs(line_position) > CURVE_THRESHOLD) {
    speed = CURVE_SPEED;        // Slow down
    kp = LINE_PID_KP * 1.2;     // More aggressive
} else {
    speed = STRAIGHT_SPEED;
    kp = LINE_PID_KP;
}
```

### IMU-Based Turns

**For accurate 90° turns:**
1. Read current heading
2. Calculate target = current ± 90°
3. Turn while monitoring heading
4. Slow down as approaching target
5. Stop when error < 3°

Your IMU code already does this!

## 🐛 Debugging Tips

### Enable Debug Printing

In `include/config.h`:
```c
#define DEBUG_PRINT_STATE       1   // State transitions
#define DEBUG_PRINT_SENSORS     1   // Sensor values
#define DEBUG_PRINT_MOTORS      1   // Motor commands
```

### Common Issues

**Line following doesn't work on curves:**
- Increase `CURVE_SPEED` (slow down more)
- Tune `LINE_PID_KP` and `LINE_PID_KD`
- Check sensor position (too far forward?)

**Robot veers left/right:**
- Adjust `LEFT_MOTOR_MULTIPLIER` or `RIGHT_MOTOR_MULTIPLIER`
- Check wheel sizes are equal
- Calibrate motor speeds

**IMU drift:**
- Recalibrate (keep robot still)
- Check I2C connections
- Verify your IMU code handles calibration correctly

**Encoders count wrong:**
- Check interrupt setup
- Verify PPR values are correct
- Test wheel rotation direction

## 📈 Expected Performance

- **Loop rate:** 50Hz (20ms per cycle)
- **Line following:** ±5mm accuracy on straight
- **Turn accuracy:** ±3° from target
- **Obstacle detection:** 2-40cm range
- **State transitions:** <50ms response time

## 🎯 Success Criteria

### Minimum Requirements
- [ ] Follows line on straight sections
- [ ] Executes 90° turns from barcodes
- [ ] Detects and stops for obstacles
- [ ] Completes simple course

### Full Requirements
- [ ] Follows line including curves
- [ ] Accurate 90° turns (±3°)
- [ ] Avoids obstacles and returns to line
- [ ] Decodes barcode commands
- [ ] Completes complex course

### Bonus Features
- [ ] Telemetry working
- [ ] U-turn command
- [ ] Speed optimization
- [ ] Multiple obstacle handling

## 📞 Support

### If You're Stuck

1. **Check interface:** Is your function signature correct?
2. **Test standalone:** Does your driver work by itself?
3. **Enable debug:** Print sensor values / motor commands
4. **Ask team lead:** Interface unclear? Need help integrating?

### Common Questions

**Q: Where do I start?**
**A:** Read `TEAM_TASK_ASSIGNMENT.md`, pick a task, implement the interface.

**Q: How do I test my code?**
**A:** Write a simple `test_motor.c` that only uses your driver.

**Q: My code compiles but doesn't work?**
**A:** Enable debug printing, check sensor values, verify pin assignments.

**Q: How do I integrate with state machine?**
**A:** Your functions are called from `state_machine_execute()`. Check that file.

## 🎉 Final Notes

### What Makes This Professional

1. **Clean interfaces** - Each subsystem has clear responsibilities
2. **Parallel development** - Team can work independently
3. **Testable** - Each part can be tested alone
4. **Maintainable** - Easy to understand and modify
5. **Scalable** - Easy to add features

### Your Existing Code

**You already have working:**
- Motors with PWM
- Encoders with interrupts
- IMU with calibration
- Heading control
- Basic obstacle detection

**Just adapt the interfaces and integrate!**

### The Most Important Part

**LINE FOLLOWING ON CURVES** - This needs the most work!

Focus on:
1. Adaptive speed (slow on curves)
2. PID tuning (higher Kp and Kd for curves)
3. Sensor position (close to ground, near front)

---

**Good luck with your demo! 🚗💨**

Questions? Check the task assignment file or ask your team lead!