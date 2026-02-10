# RC iBUS → Sabertooth Robot Mower Controller (Raspberry Pi Pico)

Firmware for a **robot mower / differential-drive robot** using:

- Raspberry Pi Pico 2 (RP2350)
- FlySky / iBUS RC receiver
- Sabertooth 2x60 motor controller (packetized serial)
- Two drive motors (left/right)

The Pico receives RC commands over iBUS, applies deadzone + expo + slew limiting,
mixes throttle and steering into differential motor outputs, and sends commands to
the Sabertooth controller.

Safety features include:

- RC failsafe timeout
- Hardware arm switch
- Stick-gesture arming / disarming
- Slew-rate limiting
- Output clamping

This project is intended for **manual RC control of a robot mower**, with room to
add autonomy later.

---

## Features

### RC Input (iBUS)

- 115200 baud iBUS decoding
- 14-channel support
- Checksum validation
- Automatic frame resync
- 100 ms failsafe timeout

### Motor Output (Sabertooth Packet Serial)

- UART packetized serial @ 9600 baud
- 7-bit speed commands
- Neutral at 64
- Independent left/right control

### Mixer

- Differential drive mixing
- Adjustable deadzone
- Cubic expo
- Speed-dependent steering reduction
- Slew-rate limiting (acceleration control)
- Output clamping

### Safety

- RC failsafe shuts motors off
- Hardware arm channel required
- Stick arming + disarming
- Motors forced to zero when disarmed
- Emergency yaw-left disarm

---

## Hardware

### Required

- Raspberry Pi Pico (or compatible RP2040 board)
- FlySky-compatible iBUS receiver
- Sabertooth motor controller (packet serial mode)
- Two DC drive motors
- RC transmitter

---

## Wiring

### iBUS Receiver → Pico

| Signal | Pico Pin |
|--------|----------|
| iBUS TX | GPIO 1 |

Uses `uart0`.

---

### Pico → Sabertooth

| Signal | Pico Pin |
|--------|----------|
| TX     | GPIO 8   |

Uses `uart1`.

Sabertooth must be configured for:

- Packetized Serial
- 9600 baud
- Address = 128 (DIP switches)

---

## Channel Mapping

iBUS channels used:

| Channel | Purpose |
|---------|---------|
| 2 | Throttle |
| 3 | Steering |
| 6 | Arm switch |

(Zero-based indexing inside firmware.)

---

## Arming Logic

### Hardware Arm Switch

Channel 6 must be HIGH (>1950 µs) or motors are always disabled.

---

### Stick Gestures

With arm switch enabled:

### Arm

Throttle LOW, Yaw RIGHT, hold ~0.5 seconds.

### Disarm (Emergency Kill)

Throttle LOW, Yaw LEFT, hold ~0.5 seconds.

This provides a redundant motor kill independent of the arm switch.

---

## Mixer Behavior

1. RC pulse widths converted to normalized -1…+1
2. Deadzone applied
3. Expo curve applied
4. Steering reduced as speed increases
5. Differential mix: left = throttle + steering, right = throttle - steering
6. Outputs clamped to ±1
7. Slew limiter applied
8. Sent to Sabertooth

---

## Failsafe

If no valid iBUS frame is received for 100 ms:

- Motors immediately set to zero
- System disarms

---

## Configuration

Main tuning parameters:

- `DEADZONE` (default 0.05)
- `EXPO` (default 0.40)
- `TURN_AT_ZERO_SPEED` (default 1.00)
- `TURN_AT_FULL_SPEED` (default 0.35)
- `SLEW_PER_SEC` (default 2.5)
- `MAX_OUTPUT` (default 1.0)

Adjust these to taste.

---

## Building

Uses Pico SDK.

Typical flow:

1. Configure:
   - `mkdir build && cd build`
   - `cmake ..`
2. Build:
   - `make`

Flash using `picotool`, or drag the UF2 onto the Pico USB drive.

---

## Intended Use

This firmware is designed for:

- RC-controlled robot mowers
- Differential-drive robots
- Outdoor mobile platforms

It is NOT autonomous (yet).

---

## Safety Disclaimer

This drives real motors. There are spinning blades attached to said motors that can cut off your foot and not realize they hit anything. Please, don't do anything stupid.

Always:

- Test with wheels off the ground first
- Verify failsafe operation
- Keep blade power physically interlocked
- Have an accessible emergency stop
- Wear PPE

You are responsible for safe operation.

---

## Future Ideas

- Autonomous mowing logic
- GPS boundary support
- IMU stabilization
- CAN bus support
- Telemetry feedback

---

## License

The Unlicense
