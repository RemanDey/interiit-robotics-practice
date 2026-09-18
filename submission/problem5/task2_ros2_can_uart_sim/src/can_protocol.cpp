#include "can_protocol.hpp"
#include <iostream>
#include <cerrno>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace rover_hardware {

SocketCanDriver::~SocketCanDriver() {
    closeInterface();
}

bool SocketCanDriver::openInterface(const std::string& interface_name) {
    m_interface_name = interface_name;

    m_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket_fd >= 0) {
        int flags = fcntl(m_socket_fd, F_GETFL, 0);
        if (flags >= 0) fcntl(m_socket_fd, F_SETFL, flags | O_NONBLOCK);

        ifreq ifr{};
        std::strncpy(ifr.ifr_name, m_interface_name.c_str(), IFNAMSIZ - 1);
        if (ioctl(m_socket_fd, SIOCGIFINDEX, &ifr) >= 0) {
            sockaddr_can addr{};
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (bind(m_socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) >= 0) {
                std::cout << "[SocketCAN] Bound successfully to native CAN interface '" << m_interface_name << "'." << std::endl;
                m_is_udp_fallback = false;
                return true;
            }
        }
        close(m_socket_fd);
        m_socket_fd = -1;
    }

    // Fallback: Virtual UDP Socket Loopback (Port 9090 <-> Port 9091)
    std::cout << "[SocketCAN] Native CAN interface '" << m_interface_name 
              << "' unavailable. Opening Virtual UDP CAN fallback (127.0.0.1:9090)..." << std::endl;

    m_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_socket_fd < 0) return false;

    int flags = fcntl(m_socket_fd, F_GETFL, 0);
    if (flags >= 0) fcntl(m_socket_fd, F_SETFL, flags | O_NONBLOCK);

    int opt = 1;
    setsockopt(m_socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    local_addr.sin_port = htons(9090);

    if (bind(m_socket_fd, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
        close(m_socket_fd);
        m_socket_fd = -1;
        return false;
    }

    m_is_udp_fallback = true;
    std::cout << "[SocketCAN] Virtual UDP CAN fallback active." << std::endl;
    return true;
}

void SocketCanDriver::closeInterface() {
    if (m_socket_fd >= 0) {
        close(m_socket_fd);
        m_socket_fd = -1;
    }
}

bool SocketCanDriver::sendMotorVelocity(uint8_t motor_id, float velocity) {
    if (m_socket_fd < 0) return false;

    CanMotorCmdFrame cmd{motor_id, velocity, 0};
    can_frame frame{};
    cmd.pack(frame);

    if (m_is_udp_fallback) {
        sockaddr_in dest_addr{};
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        dest_addr.sin_port = htons(9091);

        ssize_t bytes_written = sendto(m_socket_fd, &frame, sizeof(can_frame), 0,
                                       reinterpret_cast<sockaddr*>(&dest_addr), sizeof(dest_addr));
        return (bytes_written == sizeof(can_frame));
    } else {
        ssize_t bytes_written = write(m_socket_fd, &frame, sizeof(can_frame));
        return (bytes_written == sizeof(can_frame));
    }
}

bool SocketCanDriver::readEncoderFeedback(CanEncoderFeedbackFrame& feedback) {
    if (m_socket_fd < 0) return false;

    can_frame frame{};
    ssize_t bytes_read = 0;

    if (m_is_udp_fallback) {
        bytes_read = recv(m_socket_fd, &frame, sizeof(can_frame), 0);
    } else {
        bytes_read = read(m_socket_fd, &frame, sizeof(can_frame));
    }

    if (bytes_read > 0) {
        return CanEncoderFeedbackFrame::unpack(frame, feedback);
    }
    return false;
}

} // namespace rover_hardware
