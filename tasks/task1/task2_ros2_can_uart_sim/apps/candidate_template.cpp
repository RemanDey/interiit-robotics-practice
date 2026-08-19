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

        // --------------------------------------------------------------------
        // TODO 1: Initialize SocketCAN (PF_CAN, SOCK_RAW, CAN_RAW) on m_can_if.
        // - Create socket using socket(PF_CAN, SOCK_RAW, CAN_RAW).
        // - Set socket flags to NON-BLOCKING mode (O_NONBLOCK).
        // - Retrieve interface index using ioctl(SIOCGIFINDEX) for m_can_if.
        // - Bind socket using bind().
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // TODO 2: Initialize POSIX Serial Port (m_serial_port) with termios.h.
        // - Open device port using open() in O_RDWR | O_NOCTTY | O_NONBLOCK mode.
        // - Configure baud rate to B115200 (cfsetospeed / cfsetispeed).
        // - Set 8N1 raw mode (CS8, no parity PARENB, 1 stop bit CSTOPB).
        // - Configure non-blocking read settings (VMIN=0, VTIME=0).
        // - Apply termios settings using tcsetattr(TCSANOW).
        // --------------------------------------------------------------------

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
        // CANDIDATE IMPLEMENTATION HERE
        return 0;
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
        // CANDIDATE IMPLEMENTATION HERE
        return false;
    }

    /**
     * TODO 5: Read Motor Encoder CAN Frame & Verify Checksum
     * - Perform a non-blocking read from m_can_fd into a struct can_frame.
     * - Verify DLC >= 8 and check Byte 7 against expected XOR checksum of Bytes 0..6.
     * - Unpack motor_id (Byte 0) and position float (Bytes 1..4).
     * - Return true if a valid frame was read, false otherwise.
     */
    bool readEncoderFeedback(MotorState& state) {
        // CANDIDATE IMPLEMENTATION HERE
        return false;
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
        // CANDIDATE IMPLEMENTATION HERE
        return false;
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
