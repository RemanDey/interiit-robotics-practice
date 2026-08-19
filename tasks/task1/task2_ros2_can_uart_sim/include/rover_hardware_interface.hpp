#ifndef ROVER_HARDWARE_INTERFACE_HPP
#define ROVER_HARDWARE_INTERFACE_HPP

#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include "can_protocol.hpp"
#include "uart_protocol.hpp"

namespace rover_hardware {

/**
 * 4WD Rover Hardware Interface Class
 * Exposes 4 Motor Command Interfaces (Velocity),
 * 4 Motor State Interfaces (Position & Velocity),
 * and 3 IMU State Interfaces (Accel X, Accel Y, Yaw).
 */
class RoverHardwareInterface {
public:
    RoverHardwareInterface() = default;
    ~RoverHardwareInterface();

    bool onInit(const std::string& can_interface = "vcan0", const std::string& serial_port = "/tmp/ttyV0");
    bool onActivate();
    bool onDeactivate();

    // High-frequency control loop methods (100 Hz - 500 Hz)
    // MUST be strictly non-blocking!
    bool read();
    bool write();

    // Command Interfaces (Target Velocities for 4 Motors)
    std::vector<double>& getCmdVelocities() { return m_cmd_velocities; }

    // State Interfaces (Positions & Velocities for 4 Motors + IMU)
    const std::vector<double>& getStatePositions() const { return m_state_positions; }
    const std::vector<double>& getStateVelocities() const { return m_state_velocities; }
    const ImuTelemetryData& getImuState() const { return m_imu_data; }

private:
    std::string m_can_interface = "vcan0";
    std::string m_serial_port = "/tmp/ttyV0";

    SocketCanDriver m_can_driver;
    UartSerialDriver m_uart_driver;

    // 4WD Motors: [FL, FR, RL, RR]
    std::vector<double> m_cmd_velocities = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> m_state_positions = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> m_state_velocities = {0.0, 0.0, 0.0, 0.0};

    // IMU Sensor State
    ImuTelemetryData m_imu_data;

    bool m_is_active = false;
};

} // namespace rover_hardware

#endif // ROVER_HARDWARE_INTERFACE_HPP
