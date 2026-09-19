#ifndef TCP_PROTOCOL_HPP
#define TCP_PROTOCOL_HPP

#include <string>
#include <sstream>
#include <iomanip>
#include "vehicle_model.hpp"
#include "occupancy_grid.hpp"

namespace ackermann_sim {

struct TelemetryData {
    uint64_t step_count = 0;
    double time_ms = 0.0;
    VehicleState state;
    bool is_colliding = false;
    bool is_goal_reached = false;

    std::string serialize() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4);
        ss << "TELEMETRY "
           << step_count << " "
           << time_ms << " "
           << state.x << " "
           << state.y << " "
           << state.yaw << " "
           << state.v << " "
           << state.delta << " "
           << (is_colliding ? 1 : 0) << " "
           << (is_goal_reached ? 1 : 0) << "\n";
        return ss.str();
    }

    static bool parse(const std::string& msg, TelemetryData& telem) {
        std::istringstream ss(msg);
        std::string tag;
        ss >> tag;
        if (tag != "TELEMETRY") return false;

        int coll, goal;
        ss >> telem.step_count >> telem.time_ms
           >> telem.state.x >> telem.state.y >> telem.state.yaw
           >> telem.state.v >> telem.state.delta
           >> coll >> goal;
        telem.is_colliding = (coll != 0);
        telem.is_goal_reached = (goal != 0);
        return true;
    }
};

struct ConfigResponse {
    ScenarioInfo info;
    VehicleParams vehicle;
    double map_width;
    double map_height;
    double resolution;
    double origin_x;
    double origin_y;
    int cols;
    int rows;

    std::string serialize() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4);
        ss << "CONFIG "
           << info.start_pose.x << " " << info.start_pose.y << " " << info.start_pose.yaw << " "
           << info.goal_pose.x << " " << info.goal_pose.y << " " << info.goal_pose.yaw << " "
           << vehicle.length << " " << vehicle.width << " " << vehicle.wheelbase << " "
           << vehicle.max_steer << " " << vehicle.max_speed << " " << vehicle.min_speed << " "
           << map_width << " " << map_height << " " << resolution << " "
           << origin_x << " " << origin_y << " "
           << cols << " " << rows << "\n";
        return ss.str();
    }

    static bool parse(const std::string& msg, ConfigResponse& cfg) {
        std::istringstream ss(msg);
        std::string tag;
        ss >> tag;
        if (tag != "CONFIG") return false;

        ss >> cfg.info.start_pose.x >> cfg.info.start_pose.y >> cfg.info.start_pose.yaw
           >> cfg.info.goal_pose.x >> cfg.info.goal_pose.y >> cfg.info.goal_pose.yaw
           >> cfg.vehicle.length >> cfg.vehicle.width >> cfg.vehicle.wheelbase
           >> cfg.vehicle.max_steer >> cfg.vehicle.max_speed >> cfg.vehicle.min_speed
           >> cfg.map_width >> cfg.map_height >> cfg.resolution
           >> cfg.origin_x >> cfg.origin_y
           >> cfg.cols >> cfg.rows;
        return true;
    }
};

struct ControlCmd {
    double target_v = 0.0;
    double target_delta = 0.0;

    std::string serialize() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4);
        ss << "CTRL " << target_v << " " << target_delta << "\n";
        return ss.str();
    }

    static bool parse(const std::string& msg, ControlCmd& cmd) {
        std::istringstream ss(msg);
        std::string tag;
        ss >> tag;
        if (tag != "CTRL") return false;
        ss >> cmd.target_v >> cmd.target_delta;
        return true;
    }
};

} // namespace ackermann_sim

#endif // TCP_PROTOCOL_HPP
