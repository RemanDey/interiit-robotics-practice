#include "rover_hardware_interface.hpp"
#include <iostream>

namespace rover_hardware {

RoverHardwareInterface::~RoverHardwareInterface() {
    onDeactivate();
}

bool RoverHardwareInterface::onInit(const std::string& can_interface, const std::string& serial_port) {
    m_can_interface = can_interface;
    m_serial_port = serial_port;

    std::cout << "[Rover Hardware Interface] Initializing hardware driver..." << std::endl;
    std::cout << "  - CAN Interface:  " << m_can_interface << std::endl;
    std::cout << "  - Serial Port:    " << m_serial_port << std::endl;

    m_cmd_velocities.assign(4, 0.0);
    m_state_positions.assign(4, 0.0);
    m_state_velocities.assign(4, 0.0);

    return true;
}

bool RoverHardwareInterface::onActivate() {
    std::cout << "[Rover Hardware Interface] Activating buses..." << std::endl;
    
    bool can_ok = m_can_driver.openInterface(m_can_interface);
    bool uart_ok = m_uart_driver.openPort(m_serial_port);

    m_is_active = can_ok && uart_ok;
    if (!m_is_active) {
        std::cerr << "[Rover Hardware Interface] Warning: Bus activation incomplete. CAN: " 
                  << (can_ok ? "OK" : "FAIL") << ", UART: " << (uart_ok ? "OK" : "FAIL") << std::endl;
    } else {
        std::cout << "[Rover Hardware Interface] Driver active and ready." << std::endl;
    }

    return m_is_active;
}

bool RoverHardwareInterface::onDeactivate() {
    m_is_active = false;
    m_can_driver.closeInterface();
    m_uart_driver.closePort();
    return true;
}

bool RoverHardwareInterface::read() {
    if (!m_is_active) return false;

    // 1. Non-blocking read of motor encoder feedback frames from vcan0
    CanEncoderFeedbackFrame encoder_frame;
    while (m_can_driver.readEncoderFeedback(encoder_frame)) {
        if (encoder_frame.valid && encoder_frame.motor_id >= 1 && encoder_frame.motor_id <= 4) {
            size_t idx = encoder_frame.motor_id - 1;
            m_state_positions[idx] = encoder_frame.measured_position;
        }
    }

    // 2. Non-blocking read of IMU telemetry ASCII NMEA sentences over UART
    ImuTelemetryData latest_imu;
    if (m_uart_driver.readImuTelemetry(latest_imu)) {
        if (latest_imu.valid) {
            m_imu_data = latest_imu;
        }
    }

    return true;
}

bool RoverHardwareInterface::write() {
    if (!m_is_active) return false;

    // Non-blocking write of target velocities to 4 motors via SocketCAN
    for (size_t i = 0; i < 4; ++i) {
        uint8_t motor_id = static_cast<uint8_t>(i + 1);
        float v_target = static_cast<float>(m_cmd_velocities[i]);
        m_can_driver.sendMotorVelocity(motor_id, v_target);
    }

    return true;
}

} // namespace rover_hardware
