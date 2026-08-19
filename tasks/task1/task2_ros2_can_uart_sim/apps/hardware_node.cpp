#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include "rover_hardware_interface.hpp"

using namespace rover_hardware;

int main(int argc, char** argv) {
    std::string can_if = "vcan0";
    std::string serial_port = "/tmp/ttyV0";

    if (argc > 1) can_if = argv[1];
    if (argc > 2) serial_port = argv[2];

    std::cout << "==========================================================" << std::endl;
    std::cout << " Inter IIT Tech Meet: Task 2 Hardware Node Executable     " << std::endl;
    std::cout << "==========================================================" << std::endl;

    RoverHardwareInterface hardware_interface;
    if (!hardware_interface.onInit(can_if, serial_port)) {
        std::cerr << "[Hardware Node] Initialization failed." << std::endl;
        return 1;
    }

    if (!hardware_interface.onActivate()) {
        std::cerr << "[Hardware Node] Activation failed. Check vcan0 and " << serial_port << std::endl;
        return 1;
    }

    std::cout << "[Hardware Node] Starting 100 Hz control loop..." << std::endl;

    double t = 0.0;
    while (true) {
        // Set target wheel velocities (sine wave actuation test)
        auto& cmds = hardware_interface.getCmdVelocities();
        double v_target = 1.5 * std::sin(t);
        cmds[0] = v_target;
        cmds[1] = v_target;
        cmds[2] = v_target;
        cmds[3] = v_target;

        // Write commands over CAN
        hardware_interface.write();

        // Read encoder feedback & IMU telemetry
        hardware_interface.read();

        const auto& positions = hardware_interface.getStatePositions();
        const auto& imu = hardware_interface.getImuState();

        std::cout << "\r[100 Hz Loop] Actuation Cmd: " << std::fixed << std::setprecision(2) << v_target
                  << " m/s | Wheel Pos [FL:" << positions[0] << " FR:" << positions[1] 
                  << "] | IMU Yaw: " << imu.yaw << " rad" << std::flush;

        t += 0.01;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    hardware_interface.onDeactivate();
    return 0;
}
