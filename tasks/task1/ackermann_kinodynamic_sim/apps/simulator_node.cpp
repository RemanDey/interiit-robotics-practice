#include <iostream>
#include <cstdlib>
#include "simulator.hpp"

using namespace ackermann_sim;

int main(int argc, char** argv) {
    uint16_t port = 8091;
    ScenarioType scenario = ScenarioType::PARALLEL_PARKING_STREET;

    if (argc > 1) {
        port = static_cast<uint16_t>(std::atoi(argv[1]));
    }
    if (argc > 2) {
        int sc_id = std::atoi(argv[2]);
        if (sc_id == 1) scenario = ScenarioType::PARALLEL_PARKING_LOT;
        else if (sc_id == 2) scenario = ScenarioType::SLALOM_COURSE;
        else if (sc_id == 3) scenario = ScenarioType::MULTI_GOAL_WAYPOINTS;
    }

    std::cout << "==========================================================" << std::endl;
    std::cout << " INTER IIT TECH MEET: Autonomous Kinodynamic Simulator   " << std::endl;
    std::cout << "==========================================================" << std::endl;
    std::cout << " Listening Port: " << port << std::endl;
    std::cout << " Usage: ./simulator_node [port] [scenario_id]" << std::endl;
    std::cout << "        Scenario 0: Street Parallel Parking" << std::endl;
    std::cout << "        Scenario 1: Parking Lot Bay" << std::endl;
    std::cout << "        Scenario 2: Slalom Track" << std::endl;
    std::cout << "        Scenario 3: Multi-Goal Navigation" << std::endl;
    std::cout << " Press ESC or 'q' in visualizer to exit." << std::endl;
    std::cout << " Manual teleop: W (forward), S (reverse), A (steer left), D (steer right), Space (brake)" << std::endl;
    std::cout << "==========================================================" << std::endl;

    GameWorld gameWorld(port, scenario);

    while (gameWorld.spinOnce()) {
        // Main physics and visualization loop
    }

    std::cout << "[Simulator] Exiting simulation." << std::endl;
    return 0;
}
