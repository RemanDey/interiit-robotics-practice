/**
 * ============================================================================
 * INTER IIT TECH MEET - TASK 2 CANDIDATE STARTER TEMPLATE
 * ============================================================================
 * Problem Statement: Custom Hardware Abstraction Driver over CAN and UART
 * Objectives:
 * 1. Implement CAN Bus Actuation (vcan0): Pack target motor velocity (Float32 LE)
 *    and XOR checksum into 8-byte CAN frame. Write using Linux SocketCAN.
 * 2. Implement CAN Encoder Feedback (vcan0): Read incoming encoder frames, verify XOR
 *    checksum, update wheel state.
 * 3. Implement UART Serial Parser (/tmp/ttyV0): Configure termios (115200 8N1 Non-blocking),
 *    parse ASCII NMEA "$ROVER,IMU,accel_x,accel_y,yaw*CRC\n", verify 8-bit XOR CRC.
 * ============================================================================
 */

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <thread>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

struct MotorCommand {
    uint8_t motor_id = 1;      // 1..4
    float target_velocity = 0.0f;
};

struct MotorState {
    uint8_t motor_id = 0;
    float position = 0.0f;
};

struct ImuData {
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float yaw = 0.0f;
};

class CandidateHardwareDriver {
public:
    CandidateHardwareDriver() = default;
    ~CandidateHardwareDriver() { closeBuses(); }

    bool init(const std::string& can_if = "vcan0", const std::string& serial_port = "/tmp/ttyV0") {
        m_can_if = can_if;
        m_serial_port = serial_port;

        std::cout << "[Candidate HAL Driver] Initializing hardware interfaces..." << std::endl;

        // TODO 1: SocketCAN non-blocking + bind
        m_can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        fcntl(m_can_fd, F_SETFL, O_NONBLOCK);
        struct ifreq ifr{}; strcpy(ifr.ifr_name, m_can_if.c_str());
        ioctl(m_can_fd, SIOCGIFINDEX, &ifr);
        struct sockaddr_can addr{}; addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
        bind(m_can_fd, (struct sockaddr*)&addr, sizeof(addr));

        // TODO 2: Serial 115200 8N1 raw non-blocking
        m_serial_fd = open(m_serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        struct termios t{}; cfmakeraw(&t); cfsetspeed(&t, B115200);
        t.c_cflag |= CREAD | CLOCAL; t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
        tcsetattr(m_serial_fd, TCSANOW, &t);

        return (m_can_fd >= 0 && m_serial_fd >= 0);
    }

    void closeBuses() {
        if (m_can_fd >= 0) { close(m_can_fd); m_can_fd = -1; }
        if (m_serial_fd >= 0) { close(m_serial_fd); m_serial_fd = -1; }
    }

    /**
     * TODO 3: Compute 8-bit XOR Checksum
     * Calculate: checksum = payload[0] ^ payload[1] ^ ... ^ payload[6]
     */
    uint8_t computeCanChecksum(const uint8_t* payload_7bytes) {
        uint8_t c = 0;
        for (int i = 0; i < 7; i++) c ^= payload_7bytes[i];
        return c;
    }

    /**
     * TODO 4: Send Motor Actuation CAN Frame
     * - Construct an 8-byte struct can_frame for CAN ID (0x100 + motor_id).
     * - Byte 0: motor_id
     * - Bytes 1..4: target_velocity (Float32 Little-Endian)
     * - Bytes 5..6: Reserved (0x00)
     * - Byte 7: XOR Checksum of Bytes 0..6
     * - Write non-blocking frame using write(m_can_fd, &frame, sizeof(frame)).
     */
    bool sendMotorVelocity(uint8_t motor_id, float target_velocity) {
        struct can_frame f{}; f.can_id = 0x100 + motor_id; f.can_dlc = 8;
        f.data[0] = motor_id; memcpy(&f.data[1], &target_velocity, 4);
        f.data[7] = computeCanChecksum(f.data);
        return write(m_can_fd, &f, sizeof(f)) == sizeof(f);
    }

    /**
     * TODO 5: Read Motor Encoder CAN Frame & Verify Checksum
     * - Perform a non-blocking read from m_can_fd into a struct can_frame.
     * - Verify DLC >= 8 and check Byte 7 against expected XOR checksum of Bytes 0..6.
     * - Unpack motor_id (Byte 0) and position float (Bytes 1..4).
     * - Return true if a valid frame was read, false otherwise.
     */
    bool readEncoderFeedback(MotorState& state) {
        struct can_frame f{};
        if (read(m_can_fd, &f, sizeof(f)) != sizeof(f)) return false;
        if (computeCanChecksum(f.data) != f.data[7]) return false;
        state.motor_id = f.data[0]; memcpy(&state.position, &f.data[1], 4);
        return true;
    }

    /**
     * TODO 6: Read & Parse UART NMEA IMU Sentence ($ROVER,IMU,accel_x,accel_y,yaw*CRC\n)
     * - Perform non-blocking read from m_serial_fd into an internal string buffer.
     * - Look for complete lines delimited by '\n'.
     * - Verify string starts with '$' and contains '*' separator.
     * - Extract string body between '$' and '*' and calculate 8-bit XOR CRC.
     * - Verify calculated 2-digit HEX CRC matches the provided CRC string.
     * - Parse comma-separated float tokens for accel_x, accel_y, and yaw.
     * - Return true if a valid sentence was parsed, false otherwise.
     */
    bool readImuTelemetry(ImuData& imu) {
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
    }

private:
    int m_can_fd = -1;
    int m_serial_fd = -1;
    std::string m_can_if;
    std::string m_serial_port;
    std::string m_rx_buf;
};

int main(int argc, char** argv) {
    std::string can_if = "vcan0";
    std::string serial_port = "/tmp/ttyV0";

    if (argc > 1) can_if = argv[1];
    if (argc > 2) serial_port = argv[2];

    std::cout << "==========================================================" << std::endl;
    std::cout << "  Inter IIT Tech Meet: Task 2 Candidate HAL Driver        " << std::endl;
    std::cout << "==========================================================" << std::endl;

    CandidateHardwareDriver driver;
    if (!driver.init(can_if, serial_port)) {
        std::cerr << "[HAL Driver] Driver initialization failed. Check bus setup and emulator!" << std::endl;
        return 1;
    }

    std::cout << "[HAL Driver] Driver initialized. Running control loop..." << std::endl;

    double t = 0.0;
    while (true) {
        // Actuation test
        float target_v = static_cast<float>(2.0 * std::sin(t));
        for (uint8_t id = 1; id <= 4; ++id) {
            driver.sendMotorVelocity(id, target_v);
        }

        // Read feedback
        MotorState m_st;
        while (driver.readEncoderFeedback(m_st)) {
            std::cout << "[HAL Driver] Encoder Feedback Motor " << (int)m_st.motor_id 
                      << " Pos: " << m_st.position << std::endl;
        }

        // Read IMU telemetry
        ImuData imu;
        if (driver.readImuTelemetry(imu)) {
            std::cout << "[HAL Driver] IMU Telemetry -> AccelX: " << imu.accel_x 
                      << ", AccelY: " << imu.accel_y << ", Yaw: " << imu.yaw << std::endl;
        }

        t += 0.01;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
