## Demo Video

[![Demo Video](https://img.youtube.com/vi/Mngrw9pWEhQ/hqdefault.jpg)](https://youtu.be/Mngrw9pWEhQ)

https://youtu.be/Mngrw9pWEhQ

# Task 2 — Custom Hardware Abstraction Driver (CAN & UART): Solution

## 1. Overview
Implemented `apps/candidate_template.cpp` (`CandidateHardwareDriver`) to talk directly to the Rover Firmware Emulator over virtual hardware buses:

```
[ Control loop ] -> sendMotorVelocity() -> SocketCAN vcan0 (0x101-0x104) -> Emulator
                 <- readEncoderFeedback() <- SocketCAN vcan0 (0x201-0x204) <-
                 <- readImuTelemetry()   <- UART /tmp/ttyV0 ($ROVER,IMU...*CRC) <-
```

All I/O is strictly non-blocking for 100 Hz operation. Every frame/sentence is checksum-verified before state is updated.

## 2. Protocol

### CAN (8-byte payload)
| Byte | Content |
|---|---|
| 0 | Motor ID 1..4 |
| 1..4 | float32 LE velocity (cmd) / position (feedback) |
| 5..6 | 0x00 reserved |
| 7 | XOR `Byte0 ^ ... ^ Byte6` |

Cmd ID = `0x100 + motor_id` (0x101..0x104). Feedback ID = `0x200 + motor_id` (0x201..0x204).

### UART NMEA
```
$ROVER,IMU,<ax>,<ay>,<yaw>*<CRC>\n
```
`CRC` = uppercase `%02X` of 8-bit XOR of all chars between `$` and `*`.
Example: `$ROVER,IMU,0.120,-0.050,1.5708*3E`.

## 3. Implementation (kept minimal on purpose)

### TODO 1 — SocketCAN init
```cpp
m_can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
fcntl(m_can_fd, F_SETFL, O_NONBLOCK);
struct ifreq ifr{}; strcpy(ifr.ifr_name, m_can_if.c_str());
ioctl(m_can_fd, SIOCGIFINDEX, &ifr);
struct sockaddr_can addr{}; addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
bind(m_can_fd, (struct sockaddr*)&addr, sizeof(addr));
```
Why: `SOCK_RAW` gives raw `struct can_frame` (16 B). `O_NONBLOCK` makes `read`/`write` return immediately (`-1/EAGAIN` if empty/full) as required for real-time loop. `ioctl(SIOCGIFINDEX)+bind` attaches to `vcan0`.

### TODO 2 — Serial init
```cpp
m_serial_fd = open(m_serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
struct termios t{}; cfmakeraw(&t); cfsetspeed(&t, B115200);
t.c_cflag |= CREAD | CLOCAL; t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
tcsetattr(m_serial_fd, TCSANOW, &t);
```
Why: `cfmakeraw()` is the short form for 8N1 raw (CS8, ~PARENB, 1 stop, ~ICANON/~ECHO/~ISIG, ~IXON, ~OPOST). `B115200 + CREAD|CLOCAL + VMIN=0 VTIME=0` = 115200 8N1, non-blocking polling read.

### TODO 3 — CAN checksum
```cpp
uint8_t c = 0;
for (int i = 0; i < 7; i++) c ^= payload_7bytes[i];
return c;
```

### TODO 4 — Send velocity
```cpp
struct can_frame f{}; f.can_id = 0x100 + motor_id; f.can_dlc = 8;
f.data[0] = motor_id; memcpy(&f.data[1], &target_velocity, 4);
f.data[7] = computeCanChecksum(f.data);
return write(m_can_fd, &f, sizeof(f)) == sizeof(f);
```
`memcpy` preserves LE float on x86_64. `f.data[5..6]` stay `0` from `{}`.

### TODO 5 — Read encoder
```cpp
struct can_frame f{};
if (read(m_can_fd, &f, sizeof(f)) != sizeof(f)) return false;
if (computeCanChecksum(f.data) != f.data[7]) return false;
state.motor_id = f.data[0]; memcpy(&state.position, &f.data[1], 4);
return true;
```
Caller `while(readEncoderFeedback())` drains all queued frames. Corrupt frames are dropped (`false`).

### TODO 6 — Read IMU
```cpp
char b[256]; int n = read(m_serial_fd, b, sizeof(b));
if (n > 0) m_rx_buf += std::string(b, n);
auto p = m_rx_buf.find('\n'); if (p == std::string::npos) return false;
std::string l = m_rx_buf.substr(0, p); m_rx_buf.erase(0, p + 1);
auto s = l.find('*'); if (l.empty() || l[0] != '$' || s == std::string::npos) return false;
std::string body = l.substr(1, s - 1);
uint8_t c = 0; for (char x : body) c ^= (uint8_t)x;
char e[8]; snprintf(e, sizeof(e), "%02X", c);
if (l.substr(s + 1, 2) != e) return false;
return sscanf(body.c_str(), "ROVER,IMU,%f,%f,%f", &imu.accel_x, &imu.accel_y, &imu.yaw) == 3;
```
Handles partial reads via `m_rx_buf` (leftover kept across calls), `sscanf` enforces exact `ROVER,IMU` prefix and 3 floats, strict uppercase CRC compare.

## 4. Build & Run
```bash
./setup_vcan_pty.sh
python3 firmware_emulator/rover_firmware_emulator.py   # Terminal 1
cmake -S . -B build && cmake --build build -j4          # Terminal 2
./build/candidate_template vcan0 /tmp/ttyV0
```
Expected: `Encoder Feedback Motor 1..4 Pos: ...` + `IMU Telemetry -> AccelX, AccelY, Yaw` streaming at 100 Hz. Build verified with GCC 11.4, C++17.

## 5. Design choices / limits
- Strict spec only: no UDP fallback, strict `02X` CRC, exact `ROVER,IMU` format.
- Minimal error handling in `init()` (relies on `fd >= 0` check); sufficient when `vcan0` + `socat` setup ran first.
- `readImuTelemetry()` parses one line per call; `main()` polls at 100 Hz so it keeps up with 100 Hz emulator stream. Buffer keeps partial line; add 4 KB cap if running long-term.
- Float endianness assumed LE (true on x86_64 test host).
