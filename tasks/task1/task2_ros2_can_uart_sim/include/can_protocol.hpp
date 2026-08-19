#ifndef CAN_PROTOCOL_HPP
#define CAN_PROTOCOL_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <cstring>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>

namespace rover_hardware {

/**
 * Custom CAN Motor Control Frame Structure (8 Bytes Payload)
 * - Byte 0: Motor ID (uint8_t: 1, 2, 3, 4)
 * - Bytes 1..4: Target Velocity (float32, Little-Endian)
 * - Byte 5..6: Reserved (0x00)
 * - Byte 7: XOR Checksum (Byte0 ^ Byte1 ^ Byte2 ^ Byte3 ^ Byte4 ^ Byte5 ^ Byte6)
 */
struct CanMotorCmdFrame {
    uint8_t motor_id = 1;
    float target_velocity = 0.0f;
    uint8_t checksum = 0;

    static uint8_t computeChecksum(uint8_t motor_id, float velocity) {
        uint8_t payload[8] = {0};
        payload[0] = motor_id;
        std::memcpy(&payload[1], &velocity, sizeof(float));
        uint8_t chk = 0;
        for (int i = 0; i < 7; ++i) chk ^= payload[i];
        return chk;
    }

    void pack(can_frame& frame, uint32_t can_id_base = 0x100) const {
        std::memset(&frame, 0, sizeof(can_frame));
        frame.can_id = can_id_base + motor_id;
        frame.can_dlc = 8;

        frame.data[0] = motor_id;
        std::memcpy(&frame.data[1], &target_velocity, sizeof(float));
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = computeChecksum(motor_id, target_velocity);
    }
};

/**
 * Custom CAN Encoder Feedback Frame Structure (8 Bytes Payload)
 */
struct CanEncoderFeedbackFrame {
    uint8_t motor_id = 0;
    float measured_position = 0.0f;
    bool valid = false;

    static bool unpack(const can_frame& frame, CanEncoderFeedbackFrame& feedback) {
        if (frame.can_dlc < 8) return false;

        uint8_t chk = 0;
        for (int i = 0; i < 7; ++i) chk ^= frame.data[i];
        if (chk != frame.data[7]) return false; // Checksum mismatch!

        feedback.motor_id = frame.data[0];
        std::memcpy(&feedback.measured_position, &frame.data[1], sizeof(float));
        feedback.valid = true;
        return true;
    }
};

class SocketCanDriver {
public:
    SocketCanDriver() = default;
    ~SocketCanDriver();

    bool openInterface(const std::string& interface_name = "vcan0");
    void closeInterface();

    // Non-blocking write of motor target velocity
    bool sendMotorVelocity(uint8_t motor_id, float velocity);

    // Non-blocking read of motor encoder feedback
    bool readEncoderFeedback(CanEncoderFeedbackFrame& feedback);

    bool isOpen() const { return m_socket_fd >= 0; }

private:
    int m_socket_fd = -1;
    bool m_is_udp_fallback = false;
    std::string m_interface_name;
};

} // namespace rover_hardware

#endif // CAN_PROTOCOL_HPP
