# ESP32 PID Line Follower Robot

An ESP32-based line-following robot project using a five-channel IR sensor array and PID-based control. The firmware includes line tracking, turn handling, junction sequencing, and line-loss recovery logic.

## Features

- ESP32-based robot control
- Five-channel IR sensor interface
- PID control with three gain zones
- Motor speed and direction control
- Left-priority junction handling
- 45-degree and 90-degree turn handling
- U-turn handling
- Line-loss recovery logic
- Filtered derivative and error-history tracking

## Hardware Components

- ESP32 development board
- Five-channel IR sensor array
- Motor driver module compatible with the motors and power supply
- Two DC geared motors
- Robot chassis
- Suitable battery and connecting wires

Confirm the components and wiring against your actual hardware before powering the robot.

## Hardware Pin Mapping

### Motor connections

| Function | ESP32 GPIO |
|---|---:|
| Right motor PWM | 15 |
| Right motor forward | 2 |
| Right motor reverse | 18 |
| Left motor PWM | 14 |
| Left motor forward | 12 |
| Left motor reverse | 13 |

### IR sensor connections

The sensors are arranged from left to right as follows.

| Sensor | ESP32 GPIO | Weight |
|---|---:|---:|
| S0 — leftmost | 34 | -1600 |
| S1 | 35 | -800 |
| S2 — center | 32 | 0 |
| S3 | 33 | 800 |
| S4 — rightmost | 25 | 1600 |

The firmware defines `LINE_DETECTED` as `LOW`. Verify that this matches the output behavior of your sensor module.

## PID Parameters

The firmware uses different proportional and derivative gains for straight, curve, and sharp-turn zones.

| Parameter | Value |
|---|---|
| Proportional gains (KP) | 0.08, 0.30, 0.55 |
| Derivative gains (KD) | 0.65, 2.80, 5.50 |
| Integral gain (KI) | 0.00009 |
| Derivative filter alpha | 0.58 |
| Straight-zone correction limit | 48 |
| Curve-zone correction limit | 120 |
| Sharp-zone correction limit | 230 |
| Curve-zone threshold | 300 |
| Sharp-zone threshold | 800 |

These are the configured firmware values. They are not a claim of independently measured performance.

## Software Requirements

- Arduino IDE
- ESP32 board support in Arduino IDE
- USB data cable and the appropriate USB driver, if required by your development board

## Getting Started

1. Clone or download this repository.
2. Open `line_follower.ino` in Arduino IDE.
3. Select the appropriate ESP32 board and COM port.
4. Check the pin mapping against your circuit.
5. Confirm that the motor driver, sensors, and power supply are connected correctly.
6. Upload the firmware.
7. Test the robot on a suitable track and tune the parameters if necessary.

**Safety:** Raise the wheels during the first motor-direction test. Keep hands clear of moving parts, and verify the power supply and motor-driver ratings before operation.

## How It Works

1. The ESP32 reads the five IR sensor inputs.
2. Sensor states are used to estimate the robot's position relative to the line.
3. PID control calculates a correction based on the tracking error.
4. The firmware adjusts the motor commands to steer the robot.
5. Turn-confirmation counters and junction sequencing handle intersections and turns.
6. If the line is lost, recovery logic attempts to reacquire it.

The exact behavior depends on the sensor readings, track layout, and configured timing and speed values.

## Project Structure

```text
pid-line-follower-robot/
├── README.md
└── line_follower.ino
```

## Testing and Results

Add photographs of the assembled robot, a wiring diagram, and a demonstration video here after collecting them.

Record actual test conditions and results, such as line-following stability, junction handling, and recovery behavior. Do not include unverified performance figures.

## Future Improvements

- Add a wiring diagram and hardware photographs.
- Include a demonstration video.
- Document measured test results.
- Add troubleshooting guidance based on hardware testing.

## License

No license has been specified yet. Add an appropriate open-source license if you intend to permit others to reuse or modify this project.
