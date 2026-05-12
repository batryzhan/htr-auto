# HubTech DECODE Autonomous

FTC robot code for the 2025-2026 DECODE season.

## Hardware Configuration

- **Drive**: 4-wheel Mecanum (FL, FR, BL, BR)
- **Odometry**: 3 Dead Wheels for 2D localization
- **Intake**: DC motor for artifact collection
- **Shooter**: Flywheel motor (launcher) with encoder
- **Gate**: Servo for Overflow release to Secret Tunnel
- **Vision**: Webcam for AprilTag/Motif detection on Obelisk
- **Sensors**: Distance sensor for Classifier Ramp status

## Project Structure

```
src/
  org/firstinspires/ftc/teamcode/
    HardwarePushbot.java     # Hardware mapping class
    AutonomousDECODE.java    # Main autonomous OpMode
    TrajectoryLoader.java     # Road Runner trajectories
    Vision/
      MotifDetector.java      # VisionPortal AprilTag detection
build.gradle                 # FTC SDK and dependencies
```

## Dependencies

- FTC Robot Server 9.0.1
- Road Runner 0.5.6
- OpenCV 4.8.0

## Autonomous Sequence

1. **INIT**: Initialize motors, servos, sensors
2. **DETECT_ZONE**: Vision detects Motif pattern on Obelisk
3. **SELECT_TRAJECTORY**: Choose Left/Center/Right path
4. **PRELOAD_SCORING**: Score pre-loaded artifact
5. **INTAKE_LOOP**: Collect 3 artifacts (non-blocking while moving)
6. **GATE_CHECK**: Check classifier ramp, trigger gate if full
7. **SCORE_ARTIFACTS**: Launch collected artifacts
8. **PARK**: Navigate to Base Zone for endgame bonus

## State Machine

Enum-based state machine in `AutonomousDECODE.java`:
- Non-blocking parallel intake while driving
- PIDF launcher controller with voltage compensation

## Building

```bash
./gradlew build
```

## Hardware Names

Match these in Robot Controller app configuration:
- frontLeft, frontRight, backLeft, backRight
- deadWheelLeft, deadWheelRight, deadWheelCenter
- intakeMotor, launcher, gateServo
- distanceSensor, webcam