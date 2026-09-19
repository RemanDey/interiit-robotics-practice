/**
 * ============================================================================
 * INTER IIT TECH MEET - CANDIDATE STARTER CLIENT TEMPLATE
 * ============================================================================
 * Scenario: Autonomous Ackermann Vehicle Path Planning & Control
 * Objective: Connect to the TCP simulator, query scenario metadata, solve 
 *            path planning/navigation, and stream control commands (v, delta).
 * ============================================================================
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

struct VehicleParams {
    double length = 4.0;
    double width = 1.8;
    double wheelbase = 2.5;
    double max_steer = 0.60; // rad
    double max_speed = 2.5;  // m/s
    double min_speed = -1.5; // m/s
};

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double delta = 0.0;
};

class CandidateSolver {
public:
    CandidateSolver() {}

    /**
     * @param start Initial vehicle pose (x, y, yaw)
     * @param goal Target vehicle pose (x, y, yaw)
     * @param grid 1D vector representing binary occupancy grid (0=Free, 1=Obstacle)
     * @param cols Grid columns count
     * @param rows Grid rows count
     * @param res Grid cell resolution in meters (0.2m)
     * @param params Vehicle physical dimensions and steering limits
     * @return std::vector<Pose2D> Planned trajectory
     */
    std::vector<Pose2D> solve(const Pose2D& start, const Pose2D& goal, 
                              const std::vector<uint8_t>& grid, int cols, int rows, double res,
                              const VehicleParams& params) {
        std::cout << "[Candidate Template] Running solver..." << std::endl;
        std::vector<Pose2D> path;

        // --------------------------------------------------------------------
        // TODO: IMPLEMENT YOUR PATH PLANNING / TRAJECTORY GENERATION ALGORITHM
        // --------------------------------------------------------------------

        // Placeholder trajectory
        int num_pts = 50;
        for (int i = 0; i <= num_pts; ++i) {
            double ratio = static_cast<double>(i) / num_pts;
            Pose2D p;
            p.x = start.x + ratio * (goal.x - start.x);
            p.y = start.y + ratio * (goal.y - start.y);
            p.yaw = start.yaw + ratio * (goal.yaw - start.yaw);
            p.v = 0.5;
            p.delta = 0.0;
            path.push_back(p);
        }

        return path;
    }
};

int main(int argc, char** argv) {
    std::string ip = "127.0.0.1";
    int port = 8091;

    if (argc > 1) ip = argv[1];
    if (argc > 2) port = std::atoi(argv[2]);

    std::cout << "==========================================================" << std::endl;
    std::cout << "      Inter IIT Tech Meet: Candidate Client Template     " << std::endl;
    std::cout << "==========================================================" << std::endl;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

    if (connect(sock, reinterpret_cast<sockaddr*>(&serv_addr), sizeof(serv_addr)) < 0) {
        std::cerr << "[Client] Connection failed. Is simulator_node running on port " << port << "?" << std::endl;
        return 1;
    }

    std::cout << "[Client] Connected to simulator server!" << std::endl;

    // Send query
    std::string query = "Q\n";
    write(sock, query.c_str(), query.length());

    char buffer[16384];
    ssize_t bytes = read(sock, buffer, sizeof(buffer) - 1);
    if (bytes <= 0) return 1;
    buffer[bytes] = '\0';

    Pose2D start, goal;
    VehicleParams v_params;
    double map_w, map_h, res, orig_x, orig_y;
    int cols = 0, rows = 0;
    std::vector<uint8_t> grid;

    std::string msg(buffer);
    std::istringstream ss(msg);
    std::string line;

    while (std::getline(ss, line)) {
        if (line.rfind("CONFIG", 0) == 0) {
            std::istringstream line_ss(line.substr(7));
            line_ss >> start.x >> start.y >> start.yaw
                    >> goal.x >> goal.y >> goal.yaw
                    >> v_params.length >> v_params.width >> v_params.wheelbase
                    >> v_params.max_steer >> v_params.max_speed >> v_params.min_speed
                    >> map_w >> map_h >> res >> orig_x >> orig_y
                    >> cols >> rows;
        }
        else if (line.rfind("GRID", 0) == 0) {
            std::istringstream line_ss(line.substr(5));
            size_t count;
            line_ss >> count;
            int val;
            while (line_ss >> val) grid.push_back(val ? 1 : 0);
        }
    }

    std::cout << "[Client] Config loaded. Map: " << cols << "x" << rows << " resolution: " << res << "m" << std::endl;

    CandidateSolver solver;
    auto path = solver.solve(start, goal, grid, cols, rows, res, v_params);

    size_t target_idx = 0;
    while (true) {
        bytes = read(sock, buffer, sizeof(buffer) - 1);
        if (bytes <= 0) break;
        buffer[bytes] = '\0';

        std::string telem_str(buffer);
        if (telem_str.find("TELEMETRY") != std::string::npos) {
            std::istringstream t_ss(telem_str);
            std::string tag;
            uint64_t step;
            double t_ms, cur_x, cur_y, cur_yaw, cur_v, cur_delta;
            int coll, goal_done;
            t_ss >> tag >> step >> t_ms >> cur_x >> cur_y >> cur_yaw >> cur_v >> cur_delta >> coll >> goal_done;

            if (coll) { std::cout << "[Client] Collision detected!" << std::endl; break; }
            if (goal_done) { std::cout << "[Client] Goal reached!" << std::endl; break; }

            if (target_idx < path.size()) {
                std::ostringstream cmd_ss;
                cmd_ss << "CTRL " << path[target_idx].v << " " << path[target_idx].delta << "\n";
                std::string cmd_str = cmd_ss.str();
                write(sock, cmd_str.c_str(), cmd_str.length());
                target_idx++;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(sock);
    return 0;
}
