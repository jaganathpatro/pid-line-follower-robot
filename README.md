# ESP32 PID Line Follower Robot

An ESP32-based line-following robot project using PID control to follow a marked path. The project includes motor control, IR sensor-based line tracking, junction handling, and line-loss recovery.

## Features

- ESP32-based robot control
- PID-based line-following logic
- 5-channel IR sensor interface
- Motor speed and direction control
- Junction detection and handling
- Line-loss recovery logic

## Hardware Components

- ESP32 development board
- 5-channel IR sensor array
- Motor driver module
- DC geared motors
- Robot chassis
- Battery and connecting wires

*Confirm that the listed components match your actual hardware before building the robot.*

## Software Requirements

- Arduino IDE
- ESP32 board support package
- The appropriate libraries required by the source code

## Getting Started

1. Clone or download this repository.
2. Open `line_follower.ino` in Arduino IDE.
3. Select the appropriate ESP32 board and COM port.
4. Verify the sensor and motor pin assignments against your wiring.
5. Connect the hardware and upload the program.
6. Test the robot on a suitable line-following track.

## PID Control

PID control uses three terms:

- **Proportional (P):** responds to the current line-tracking error.
- **Integral (I):** accounts for accumulated error.
- **Derivative (D):** responds to changes in error.

The controller uses sensor readings to adjust motor behavior and help the robot stay on the track.

## Project Structure

```text
pid-line-follower-robot/
└── line_follower.ino
```

## Testing

Add your own photos, videos, and measured test results after testing the robot. Do not report performance figures until they have been verified.

## Future Improvements

- Add a wiring diagram and hardware photographs.
- Document the sensor and motor pin mapping.
- Record PID tuning parameters and test results.
- Add a demonstration video.

## License

A license has not yet been specified. Add a suitable open-source license if you want others to reuse the project.
