#ifndef UART_PROTOCOL_HPP
#define UART_PROTOCOL_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace rover_hardware {

struct ImuTelemetryData {
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float yaw = 0.0f;
    bool valid = false;
};

class UartSerialDriver {
public:
    UartSerialDriver() = default;
    ~UartSerialDriver();

    // Open POSIX Serial Port (termios 115200 baud, 8N1, Non-blocking)
    bool openPort(const std::string& port_path = "/tmp/ttyV0", speed_t baud_rate = B115200);
    void closePort();

    // Non-blocking read and line buffering for NMEA sentence parsing
    bool readImuTelemetry(ImuTelemetryData& imu_data);

    // Compute NMEA 8-bit XOR checksum string (2 hex chars)
    static std::string computeNmeaCrc(const std::string& sentence_body);

    // Parse string "$ROVER,IMU,accel_x,accel_y,yaw*CRC"
    static bool parseNmeaSentence(const std::string& line, ImuTelemetryData& imu_data);

    bool isOpen() const { return m_serial_fd >= 0; }

private:
    int m_serial_fd = -1;
    std::string m_port_path;
    std::string m_rx_buffer;
};

} // namespace rover_hardware

#endif // UART_PROTOCOL_HPP
