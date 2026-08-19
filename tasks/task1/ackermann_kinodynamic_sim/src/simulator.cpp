#include "simulator.hpp"
#include <iostream>
#include <sstream>
#include <cmath>

namespace ackermann_sim {

uint64_t GameWorld::getCurrentTimeMs() const {
    return static_cast<uint64_t>(static_cast<double>(cv::getTickCount()) / cv::getTickFrequency() * 1000.0);
}

GameWorld::GameWorld(uint16_t tcp_port, ScenarioType scenario)
    : M_GAME_START_TIME_MS(getCurrentTimeMs()), m_tcpPort(tcp_port) {

    m_scenario_info = m_grid.loadScenario(scenario);
    m_state = m_scenario_info.start_pose;
    m_state_history.push_back({m_state.x, m_state.y});

    setupTcpServer();
    std::cout << "[Simulator] World initialized. Scenario: " << m_scenario_info.name << std::endl;
    std::cout << "[Simulator] Start Pose: (" << m_state.x << ", " << m_state.y << ", " << m_state.yaw << ")" << std::endl;
    std::cout << "[Simulator] Goal Pose:  (" << m_scenario_info.goal_pose.x << ", " << m_scenario_info.goal_pose.y 
              << ", " << m_scenario_info.goal_pose.yaw << ")" << std::endl;
}

GameWorld::~GameWorld() {
    if (m_tcpClientSocketFd != -1) close(m_tcpClientSocketFd);
    if (m_tcpServerSocketFd != -1) close(m_tcpServerSocketFd);
    cv::destroyAllWindows();
}

void GameWorld::setupTcpServer() {
    m_tcpServerSocketFd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_tcpServerSocketFd < 0) {
        perror("[TCP] Error creating socket");
        exit(EXIT_FAILURE);
    }

    int flags = fcntl(m_tcpServerSocketFd, F_GETFL, 0);
    fcntl(m_tcpServerSocketFd, F_SETFL, flags | O_NONBLOCK);

    int opt = 1;
    setsockopt(m_tcpServerSocketFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(m_tcpPort);

    if (bind(m_tcpServerSocketFd, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        perror("[TCP] Error binding socket");
        exit(EXIT_FAILURE);
    }

    if (listen(m_tcpServerSocketFd, 2) < 0) {
        perror("[TCP] Error listening");
        exit(EXIT_FAILURE);
    }

    std::cout << "[TCP Server] Listening on port " << m_tcpPort << std::endl;
}

void GameWorld::handleTcpClient() {
    if (m_tcpClientSocketFd == -1) {
        m_tcpClientSocketFd = accept(m_tcpServerSocketFd, nullptr, nullptr);
        if (m_tcpClientSocketFd > 0) {
            std::cout << "[TCP Server] Client connected!" << std::endl;
            int flags = fcntl(m_tcpClientSocketFd, F_GETFL, 0);
            fcntl(m_tcpClientSocketFd, F_SETFL, flags | O_NONBLOCK);
        }
        return;
    }

    char buffer[4096];
    ssize_t bytes_read = read(m_tcpClientSocketFd, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        std::string msg(buffer);
        std::istringstream stream(msg);
        std::string line;

        while (std::getline(stream, line)) {
            if (line.empty()) continue;

            if (line[0] == 'Q' || line.rfind("GET_CONFIG", 0) == 0) {
                std::cout << "[TCP Server] Query received from client." << std::endl;
                ConfigResponse cfg;
                cfg.info = m_scenario_info;
                cfg.vehicle = m_vehicle_model.getParams();
                cfg.map_width = m_grid.getWidth();
                cfg.map_height = m_grid.getHeight();
                cfg.resolution = m_grid.getResolution();
                cfg.origin_x = m_grid.getOriginX();
                cfg.origin_y = m_grid.getOriginY();
                cfg.cols = m_grid.getCols();
                cfg.rows = m_grid.getRows();

                std::string resp = cfg.serialize();
                write(m_tcpClientSocketFd, resp.c_str(), resp.length());

                const auto& grid_data = m_grid.getGridData();
                std::ostringstream ss_grid;
                ss_grid << "GRID " << grid_data.size();
                for (uint8_t val : grid_data) {
                    ss_grid << " " << (val > 127 ? 1 : 0);
                }
                ss_grid << "\n";
                std::string grid_str = ss_grid.str();
                write(m_tcpClientSocketFd, grid_str.c_str(), grid_str.length());
                std::cout << "[TCP Server] Sent scenario configuration & occupancy grid." << std::endl;
            }
            else if (line.rfind("CTRL", 0) == 0) {
                ControlCmd cmd;
                if (ControlCmd::parse(line, cmd)) {
                    m_current_cmd = cmd;
                    m_manual_control = false;
                }
            }
            else if (line.rfind("TRAJ", 0) == 0) {
                std::istringstream ss_traj(line.substr(5));
                std::string pt_str;
                m_planned_path.clear();
                while (std::getline(ss_traj, pt_str, ';')) {
                    if (pt_str.empty()) continue;
                    std::istringstream ss_pt(pt_str);
                    VehicleState st;
                    if (ss_pt >> st.x >> st.y >> st.yaw >> st.v) {
                        m_planned_path.push_back(st);
                    }
                }
                std::cout << "[TCP Server] Received trajectory with " 
                          << m_planned_path.size() << " waypoints." << std::endl;
            }
        }
    }
    else if (bytes_read == 0) {
        std::cout << "[TCP Server] Client disconnected." << std::endl;
        close(m_tcpClientSocketFd);
        m_tcpClientSocketFd = -1;
    }
}

bool GameWorld::checkGoalReached() const {
    if (!m_scenario_info.waypoints.empty()) {
        for (const auto& wp : m_scenario_info.waypoints) {
            if (!wp.visited) return false;
        }
        return true;
    }

    const auto& goal = m_scenario_info.goal_pose;
    double dx = m_state.x - goal.x;
    double dy = m_state.y - goal.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    double dyaw = std::abs(m_state.yaw - goal.yaw);
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    dyaw = std::abs(dyaw);

    return (dist < 0.40 && dyaw < 0.30 && std::abs(m_state.v) < 0.2);
}

void GameWorld::setPlannedPath(const std::vector<VehicleState>& path) {
    m_planned_path = path;
}

bool GameWorld::spinOnce() {
    handleTcpClient();

    if (!m_collision_occurred && !m_goal_reached) {
        m_state = m_vehicle_model.step(m_state, m_current_cmd.target_v, m_current_cmd.target_delta, m_dt);
        m_step_count++;

        if (m_state_history.empty() || 
            std::hypot(m_state.x - m_state_history.back().x, m_state.y - m_state_history.back().y) > 0.1) {
            m_state_history.push_back({m_state.x, m_state.y});
        }

        // Waypoint check
        for (size_t i = 0; i < m_scenario_info.waypoints.size(); ++i) {
            auto& wp = m_scenario_info.waypoints[i];
            if (!wp.visited) {
                double dist = std::hypot(m_state.x - wp.x, m_state.y - wp.y);
                if (dist < 1.2) {
                    wp.visited = true;
                    std::cout << "[Simulator] Waypoint #" << (i+1) << " reached at (" 
                              << wp.x << ", " << wp.y << ")" << std::endl;
                }
            }
        }

        auto corners = m_vehicle_model.getBoundingBoxCorners(m_state);
        m_collision_occurred = m_grid.isVehicleColliding(corners);
        if (m_collision_occurred) {
            std::cout << "[Simulator] Collision detected at step " << m_step_count << "!" << std::endl;
        }

        m_goal_reached = checkGoalReached();
        if (m_goal_reached) {
            std::cout << "[Simulator] Goal reached in " << m_step_count << " steps (" 
                      << m_step_count * m_dt << "s)!" << std::endl;
        }
    }

    if (m_tcpClientSocketFd > 0) {
        TelemetryData telem;
        telem.step_count = m_step_count;
        telem.time_ms = m_step_count * m_dt * 1000.0;
        telem.state = m_state;
        telem.is_colliding = m_collision_occurred;
        telem.is_goal_reached = m_goal_reached;

        std::string msg = telem.serialize();
        write(m_tcpClientSocketFd, msg.c_str(), msg.length());
    }

    return renderGameWindow();
}

bool GameWorld::renderGameWindow() {
    cv::Mat canvas(M_WINDOW_SIZE, M_WINDOW_SIZE, CV_8UC3, cv::Scalar(30, 32, 38));

    double map_w = m_grid.getWidth();
    double map_h = m_grid.getHeight();
    double orig_x = m_grid.getOriginX();
    double orig_y = m_grid.getOriginY();

    auto worldToCanvas = [&](double wx, double wy) -> cv::Point {
        int cx = static_cast<int>((wx - orig_x) / map_w * M_WINDOW_SIZE);
        int cy = static_cast<int>(M_WINDOW_SIZE - (wy - orig_y) / map_h * M_WINDOW_SIZE);
        return cv::Point(cx, cy);
    };

    // 1. Grid lines
    int grid_step = static_cast<int>(5.0 / map_w * M_WINDOW_SIZE); // 5m grid lines
    for (int x = 0; x < M_WINDOW_SIZE; x += grid_step) {
        cv::line(canvas, cv::Point(x, 0), cv::Point(x, M_WINDOW_SIZE), cv::Scalar(45, 48, 56), 1);
    }
    for (int y = 0; y < M_WINDOW_SIZE; y += grid_step) {
        cv::line(canvas, cv::Point(0, y), cv::Point(M_WINDOW_SIZE, y), cv::Scalar(45, 48, 56), 1);
    }

    // 2. Obstacles
    int cols = m_grid.getCols();
    int rows = m_grid.getRows();
    double res = m_grid.getResolution();

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (m_grid.isCellOccupied(c, r)) {
                double wx, wy;
                m_grid.gridToWorld(c, r, wx, wy);
                cv::Point p1 = worldToCanvas(wx - res/2, wy + res/2);
                cv::Point p2 = worldToCanvas(wx + res/2, wy - res/2);
                cv::rectangle(canvas, p1, p2, cv::Scalar(100, 100, 120), cv::FILLED);
            }
        }
    }

    // 3. Sequential Waypoints (if scenario 3)
    for (size_t i = 0; i < m_scenario_info.waypoints.size(); ++i) {
        const auto& wp = m_scenario_info.waypoints[i];
        cv::Point pt = worldToCanvas(wp.x, wp.y);
        cv::Scalar col = wp.visited ? cv::Scalar(50, 220, 50) : cv::Scalar(220, 50, 220);
        cv::circle(canvas, pt, 10, col, wp.visited ? cv::FILLED : 2, cv::LINE_AA);
        cv::putText(canvas, std::to_string(i+1), cv::Point(pt.x - 4, pt.y + 4), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
    }

    // 4. Streamed Candidate Path
    if (!m_planned_path.empty()) {
        for (size_t i = 0; i < m_planned_path.size() - 1; ++i) {
            cv::Point p1 = worldToCanvas(m_planned_path[i].x, m_planned_path[i].y);
            cv::Point p2 = worldToCanvas(m_planned_path[i+1].x, m_planned_path[i+1].y);
            cv::Scalar color = (m_planned_path[i].v >= 0) ? cv::Scalar(255, 200, 50) : cv::Scalar(50, 150, 255);
            cv::line(canvas, p1, p2, color, 2, cv::LINE_AA);
        }
    }

    // 5. Vehicle History Trail
    for (size_t i = 0; i < m_state_history.size(); ++i) {
        cv::Point pt = worldToCanvas(m_state_history[i].x, m_state_history[i].y);
        cv::circle(canvas, pt, 2, cv::Scalar(0, 220, 180), cv::FILLED);
    }

    // 6. Target Goal Pose Ghost
    {
        auto goal_corners = m_vehicle_model.getBoundingBoxCorners(m_scenario_info.goal_pose);
        std::vector<cv::Point> pts;
        for (const auto& c : goal_corners) pts.push_back(worldToCanvas(c.x, c.y));
        cv::polylines(canvas, pts, true, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

        cv::Point g_center = worldToCanvas(m_scenario_info.goal_pose.x, m_scenario_info.goal_pose.y);
        double g_yaw = m_scenario_info.goal_pose.yaw;
        cv::Point g_head = worldToCanvas(m_scenario_info.goal_pose.x + 2.0 * std::cos(g_yaw),
                                         m_scenario_info.goal_pose.y + 2.0 * std::sin(g_yaw));
        cv::arrowedLine(canvas, g_center, g_head, cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.3);
    }

    // 7. Current Robot Vehicle
    {
        auto corners = m_vehicle_model.getBoundingBoxCorners(m_state);
        std::vector<cv::Point> pts;
        for (const auto& c : corners) pts.push_back(worldToCanvas(c.x, c.y));

        cv::Scalar car_color = m_collision_occurred ? cv::Scalar(50, 50, 240) : 
                               (m_goal_reached ? cv::Scalar(50, 220, 50) : cv::Scalar(60, 180, 100));
        
        cv::fillConvexPoly(canvas, pts, car_color, cv::LINE_AA);
        cv::polylines(canvas, pts, true, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

        Point2D fl, fr, rl, rr;
        m_vehicle_model.getWheelPositions(m_state, fl, fr, rl, rr);
        
        auto drawWheel = [&](Point2D center, double steer_angle, bool is_front) {
            double w_len = 0.8;
            double heading = m_state.yaw + (is_front ? steer_angle : 0.0);
            cv::Point p1 = worldToCanvas(center.x - w_len/2 * std::cos(heading), center.y - w_len/2 * std::sin(heading));
            cv::Point p2 = worldToCanvas(center.x + w_len/2 * std::cos(heading), center.y + w_len/2 * std::sin(heading));
            cv::line(canvas, p1, p2, cv::Scalar(10, 10, 10), 4, cv::LINE_AA);
        };
        drawWheel(fl, m_state.delta, true);
        drawWheel(fr, m_state.delta, true);
        drawWheel(rl, m_state.delta, false);
        drawWheel(rr, m_state.delta, false);

        cv::Point p_fl = worldToCanvas(corners[0].x, corners[0].y);
        cv::Point p_fr = worldToCanvas(corners[1].x, corners[1].y);
        cv::circle(canvas, p_fl, 4, cv::Scalar(200, 255, 255), cv::FILLED);
        cv::circle(canvas, p_fr, 4, cv::Scalar(200, 255, 255), cv::FILLED);
    }

    // 8. HUD Header Overlay
    cv::rectangle(canvas, cv::Point(0, 0), cv::Point(M_WINDOW_SIZE, 60), cv::Scalar(15, 18, 24), cv::FILLED);
    cv::line(canvas, cv::Point(0, 60), cv::Point(M_WINDOW_SIZE, 60), cv::Scalar(80, 80, 100), 1);

    char hud_title[128];
    snprintf(hud_title, sizeof(hud_title), "%s | Step: %lu | Time: %.2fs", 
             m_scenario_info.name.c_str(), m_step_count, m_step_count * m_dt);
    cv::putText(canvas, hud_title, cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // INTER IIT TECH MEET Header (Top-Right)
    cv::putText(canvas, "INTER IIT TECH MEET", cv::Point(M_WINDOW_SIZE - 230, 25), 
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);

    char hud_telemetry[128];
    snprintf(hud_telemetry, sizeof(hud_telemetry), "Pose: (%.2fm, %.2fm, %.1f deg) | Speed: %+.2fm/s | Steer: %+.1f deg", 
             m_state.x, m_state.y, m_state.yaw * 180.0 / M_PI, m_state.v, m_state.delta * 180.0 / M_PI);
    cv::putText(canvas, hud_telemetry, cv::Point(15, 48), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 220, 255), 1, cv::LINE_AA);

    // Status Banner at bottom
    cv::rectangle(canvas, cv::Point(0, M_WINDOW_SIZE - 40), cv::Point(M_WINDOW_SIZE, M_WINDOW_SIZE), cv::Scalar(15, 18, 24), cv::FILLED);
    
    std::string status_text;
    cv::Scalar status_color;
    if (m_collision_occurred) {
        status_text = "STATUS: CRASHED INTO OBSTACLE!";
        status_color = cv::Scalar(50, 50, 255);
    } else if (m_goal_reached) {
        status_text = "STATUS: GOAL REACHED!";
        status_color = cv::Scalar(50, 255, 100);
    } else if (m_tcpClientSocketFd > 0) {
        status_text = m_manual_control ? "STATUS: MANUAL TELEOP CONTROL (Keyboard W/A/S/D)" : "STATUS: CLIENT AUTO CONTROL (TCP Streaming)";
        status_color = cv::Scalar(255, 200, 50);
    } else {
        status_text = "STATUS: WAITING FOR CLIENT TCP CONNECTION (Port " + std::to_string(m_tcpPort) + ")...";
        status_color = cv::Scalar(100, 200, 255);
    }
    cv::putText(canvas, status_text, cv::Point(15, M_WINDOW_SIZE - 15), cv::FONT_HERSHEY_SIMPLEX, 0.55, status_color, 1, cv::LINE_AA);

    cv::imshow(M_WIN_NAME, canvas);

    int key = cv::waitKey(static_cast<int>(m_dt * 1000.0));
    if (key == 27 || key == 'q' || key == 'Q') {
        return false;
    }

    if (key == 'w' || key == 'W') { m_current_cmd.target_v = 1.0; m_manual_control = true; }
    else if (key == 's' || key == 'S') { m_current_cmd.target_v = -1.0; m_manual_control = true; }
    else if (key == 'a' || key == 'A') { m_current_cmd.target_delta += 0.1; m_manual_control = true; }
    else if (key == 'd' || key == 'D') { m_current_cmd.target_delta -= 0.1; m_manual_control = true; }
    else if (key == ' ') { m_current_cmd.target_v = 0.0; m_manual_control = true; }
    else if (key == 'r' || key == 'R') {
        m_state = m_scenario_info.start_pose;
        m_state_history.clear();
        m_collision_occurred = false;
        m_goal_reached = false;
        m_step_count = 0;
        m_current_cmd = ControlCmd();
    }

    return true;
}

} // namespace ackermann_sim
