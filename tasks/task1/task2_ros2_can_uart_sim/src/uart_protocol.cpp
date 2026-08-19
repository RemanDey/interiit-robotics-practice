#include "uart_protocol.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cerrno>
#include <cstring>

namespace rover_hardware {

UartSerialDriver::~UartSerialDriver() {
    closePort();
}

bool UartSerialDriver::openPort(const std::string& port_path, speed_t baud_rate) {
    m_port_path = port_path;

    // Open port in Read/Write, Non-blocking, No CTTC control mode
    m_serial_fd = open(m_port_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_serial_fd < 0) {
        std::cerr << "[UART Serial] Error opening port '" << m_port_path << "': " << std::strerror(errno) << std::endl;
        return false;
    }

    termios tty{};
    if (tcgetattr(m_serial_fd, &tty) != 0) {
        std::cerr << "[UART Serial] Error getting termios attributes: " << std::strerror(errno) << std::endl;
        closePort();
        return false;
    }

    // Set Baud Rate to 115200
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    // 8N1 Configuration (8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;        // No Parity
    tty.c_cflag &= ~CSTOPB;        // 1 Stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 Data bits
    tty.c_cflag &= ~CRTSCTS;       // No Hardware Flow Control
    tty.c_cflag |= CREAD | CLOCAL; // Enable Receiver, Ignore Control Lines

    // Raw input mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Raw output mode
    tty.c_oflag &= ~OPOST;

    // Non-blocking read (VMIN=0, VTIME=0)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "[UART Serial] Error setting termios attributes: " << std::strerror(errno) << std::endl;
        closePort();
        return false;
    }

    std::cout << "[UART Serial] Opened serial port '" << m_port_path << "' (115200 8N1 Non-blocking)." << std::endl;
    return true;
}

void UartSerialDriver::closePort() {
    if (m_serial_fd >= 0) {
        close(m_serial_fd);
        m_serial_fd = -1;
    }
}

std::string UartSerialDriver::computeNmeaCrc(const std::string& sentence_body) {
    uint8_t crc = 0;
    for (char c : sentence_body) {
        crc ^= static_cast<uint8_t>(c);
    }
    std::ostringstream ss;
    ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(crc);
    return ss.str();
}

bool UartSerialDriver::parseNmeaSentence(const std::string& line, ImuTelemetryData& imu_data) {
    // Expected format: $ROVER,IMU,accel_x,accel_y,yaw*CRC
    if (line.empty() || line[0] != '$') return false;

    size_t star_pos = line.find('*');
    if (star_pos == std::string::npos) return false;

    std::string body = line.substr(1, star_pos - 1);
    std::string provided_crc = line.substr(star_pos + 1);

    // Remove trailing \r or \n from CRC
    while (!provided_crc.empty() && (provided_crc.back() == '\r' || provided_crc.back() == '\n')) {
        provided_crc.pop_back();
    }

    std::string expected_crc = computeNmeaCrc(body);
    if (provided_crc != expected_crc) {
        std::cerr << "[UART Serial] NMEA CRC mismatch! Received: " << provided_crc 
                  << ", Expected: " << expected_crc << std::endl;
        return false;
    }

    std::istringstream ss(body);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    // Expected tokens: ["ROVER", "IMU", "accel_x", "accel_y", "yaw"]
    if (tokens.size() < 5 || tokens[0] != "ROVER" || tokens[1] != "IMU") {
        return false;
    }

    try {
        imu_data.accel_x = std::stof(tokens[2]);
        imu_data.accel_y = std::stof(tokens[3]);
        imu_data.yaw = std::stof(tokens[4]);
        imu_data.valid = true;
        return true;
    } catch (...) {
        return false;
    }
}

bool UartSerialDriver::readImuTelemetry(ImuTelemetryData& imu_data) {
    if (m_serial_fd < 0) return false;

    char chunk[256];
    ssize_t bytes_read = read(m_serial_fd, chunk, sizeof(chunk) - 1);

    if (bytes_read > 0) {
        chunk[bytes_read] = '\0';
        m_rx_buffer.append(chunk, bytes_read);

        // Process full lines ending with '\n'
        size_t newline_pos;
        bool found_valid = false;
        while ((newline_pos = m_rx_buffer.find('\n')) != std::string::npos) {
            std::string line = m_rx_buffer.substr(0, newline_pos);
            m_rx_buffer.erase(0, newline_pos + 1);

            if (!line.empty() && line.back() == '\r') line.pop_back();

            if (parseNmeaSentence(line, imu_data)) {
                found_valid = true;
            }
        }
        return found_valid;
    }

    return false;
}

} // namespace rover_hardware
