# Inter IIT Tech Meet: Task 2 - Custom Hardware Abstraction Driver (CAN & UART)

## Problem Overview

Standard robotics simulation frameworks often obscure physical hardware communication behind high-level plugin abstractions. In real-world autonomous vehicles, high-level control software must interface directly with embedded microcontrollers and sensors via low-level physical buses such as **CAN** and **UART (Serial)**.

In this task, candidates must write a custom C++ Hardware Abstraction Layer (HAL) driver (such as a `ros2_control SystemInterface`) that:
1. Serializes target wheel velocity commands into raw SocketCAN binary frames.
2. Deserializes incoming motor encoder feedback frames and validates checksums.
3. Parses non-blocking ASCII NMEA telemetry strings from a serial UART interface and verifies 8-bit XOR checksums.

---

## Hardware Architecture

```text
[ ROS 2 Controllers ] (Joint Trajectory / Diff Drive Controller)
         │
         ▼
[ Candidate's Custom ros2_control Hardware Interface ] (The Deliverable)
         │
         ├─> [SocketCAN / vcan0] ──────> [ Firmware Emulator ] ──> [ Rover Kinematics ]
         │                                       │
         └─> [UART / /tmp/ttyV0] ───────────────>┘
```

The system environment provides a **Firmware Emulator** script that acts as the vehicle's onboard microcontroller. It listens on `vcan0` and `/tmp/ttyV1` for commands and streams feedback telemetry back over the virtual hardware buses.

---

## Detailed Protocol Specifications

### 1. CAN Bus Protocol (Motor Actuation & Encoder Feedback)

- **Interface**: Linux SocketCAN (`vcan0`) using raw CAN sockets (`AF_CAN`, `SOCK_RAW`, `CAN_RAW`).
- **CAN ID Range**:
  - Motor Actuation Commands: `0x101` to `0x104` (Motors 1 to 4).
  - Motor Encoder Feedback: `0x201` to `0x204` (Motors 1 to 4).

#### CAN Frame Payload Specification (8 Bytes):

| Byte Offset | Data Type | Description |
| :---: | :---: | :--- |
| **Byte 0** | `uint8_t` | Motor Identifier (`1`, `2`, `3`, or `4`) |
| **Bytes 1–4** | `float` | Target / Measured Velocity (Float32, Little-Endian) |
| **Bytes 5–6** | `uint8_t[2]` | Reserved (`0x00`) |
| **Byte 7** | `uint8_t` | **XOR Checksum**: `Byte0 ^ Byte1 ^ Byte2 ^ Byte3 ^ Byte4 ^ Byte5 ^ Byte6` |

---

### 2. UART Serial Protocol (IMU Telemetry)

- **Port Settings**: Baud rate `115200`, `8N1` (8 data bits, no parity, 1 stop bit), Raw Mode.
- **Serial Device**: Candidate connects to `/tmp/ttyV0`.

#### ASCII NMEA Sentence Specification (100 Hz Stream):
```text
$ROVER,IMU,<accel_x>,<accel_y>,<yaw>*<CRC_HEX>\n
```

- **Example**: `$ROVER,IMU,0.120,-0.050,1.5708*3E\n`
- **Checksum Rule**:
  - The 2-character hexadecimal CRC is computed by taking the 8-bit XOR sum of all ASCII characters between `$` and `*` (exclusive of `$` and `*`).
  - Example: For `ROVER,IMU,0.120,-0.050,1.5708`, `CRC = 'R' ^ 'O' ^ 'V' ^ ...`

---

## Hard Real-Time Constraints

1. **Non-Blocking I/O**:
   - The hardware driver's `read()` and `write()` functions run inside a high-frequency control loop (100 Hz – 500 Hz).
   - All SocketCAN and POSIX serial calls must be strictly non-blocking (`O_NONBLOCK`, `VMIN=0, VTIME=0`).
   - Sockets must return immediately if no data frame is available.

2. **Data Integrity & Verification**:
   - Every received CAN frame must be verified against Byte 7 XOR checksum before updating wheel states.
   - Every received UART sentence must be validated against its NMEA CRC string before updating sensor states.

---

## Instructions for Candidates

Candidates are expected to populate the stubs in `apps/candidate_template.cpp`:

1. **`init()`**: Open and configure non-blocking SocketCAN on `vcan0` and POSIX termios serial interface on `/tmp/ttyV0`.
2. **`computeCanChecksum()`**: Implement 8-bit XOR checksum over CAN payload bytes 0–6.
3. **`sendMotorVelocity()`**: Construct and write 8-byte CAN motor command frames.
4. **`readEncoderFeedback()`**: Non-blocking read of incoming encoder frames and verification of checksum.
5. **`readImuTelemetry()`**: Non-blocking read and parsing of NMEA IMU telemetry sentences and verification of NMEA CRC.

---

## How to Test & Verify

### Step 1: Initialize Virtual Hardware Buses
```bash
./setup_vcan_pty.sh
```

### Step 2: Start Firmware Emulator (Terminal 1)
```bash
python3 firmware_emulator/rover_firmware_emulator.py
```

### Step 3: Compile Candidate Code (Terminal 2)
```bash
mkdir -p build && cd build
cmake ..
make -j4
```

### Step 4: Run Candidate Driver
```bash
./candidate_template vcan0 /tmp/ttyV0
```

If your implementation is correct, the terminal output will show real-time stream logs of encoder position feedback and IMU acceleration/yaw data received over the virtual buses.
