#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "vehicle_model.hpp"
#include "occupancy_grid.hpp"
#include "tcp_protocol.hpp"

namespace ackermann_sim {

class GameWorld {
public:
    explicit GameWorld(uint16_t tcp_port = 8091, ScenarioType scenario = ScenarioType::PARALLEL_PARKING_STREET);
    ~GameWorld();

    bool spinOnce();
    void setPlannedPath(const std::vector<VehicleState>& path);

    const VehicleState& getCurrentState() const { return m_state; }
    const VehicleState& getGoalState() const { return m_scenario_info.goal_pose; }
    const OccupancyGrid& getGrid() const { return m_grid; }
    const VehicleModel& getVehicleModel() const { return m_vehicle_model; }

private:
    void setupTcpServer();
    void handleTcpClient();
    bool checkGoalReached() const;
    bool renderGameWindow();
    uint64_t getCurrentTimeMs() const;

    /* Window & GUI Settings */
    const int M_WINDOW_SIZE = 800;
    const std::string M_WIN_NAME = "INTER IIT TECH MEET: Autonomous Kinodynamic Simulator";
    const uint64_t M_GAME_START_TIME_MS;

    /* Simulation World & State */
    uint16_t m_tcpPort;
    VehicleModel m_vehicle_model;
    OccupancyGrid m_grid;
    ScenarioInfo m_scenario_info;
    VehicleState m_state;
    
    /* Planned trajectory for rendering overlay */
    std::vector<VehicleState> m_planned_path;
    std::vector<Point2D> m_state_history;

    /* TCP Socket Server */
    int m_tcpServerSocketFd = -1;
    int m_tcpClientSocketFd = -1;

    /* Control Commands & Status */
    ControlCmd m_current_cmd;
    uint64_t m_step_count = 0;
    bool m_collision_occurred = false;
    bool m_goal_reached = false;
    bool m_manual_control = false;
    double m_dt = 0.05; // 20 Hz simulation rate
};

} // namespace ackermann_sim

#endif // SIMULATOR_HPP
