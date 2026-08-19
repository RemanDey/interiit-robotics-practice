#ifndef OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_HPP

#include <vector>
#include <cstdint>
#include <string>
#include <opencv2/opencv.hpp>
#include "vehicle_model.hpp"

namespace ackermann_sim {

enum class ScenarioType {
    PARALLEL_PARKING_STREET = 0,
    PARALLEL_PARKING_LOT = 1,
    SLALOM_COURSE = 2,
    MULTI_GOAL_WAYPOINTS = 3
};

struct Point2DGoal {
    double x;
    double y;
    bool visited = false;
};

struct ScenarioInfo {
    std::string name;
    VehicleState start_pose;
    VehicleState goal_pose;
    std::vector<Point2DGoal> waypoints;
};

class OccupancyGrid {
public:
    OccupancyGrid(double width_m = 100.0, double height_m = 100.0, double resolution = 0.2, 
                  double origin_x = -50.0, double origin_y = -50.0);

    double getWidth() const { return m_width_m; }
    double getHeight() const { return m_height_m; }
    double getResolution() const { return m_resolution; }
    int getCols() const { return m_cols; }
    int getRows() const { return m_rows; }
    double getOriginX() const { return m_origin_x; }
    double getOriginY() const { return m_origin_y; }

    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;

    bool isOccupied(double wx, double wy) const;
    bool isCellOccupied(int gx, int gy) const;
    void setOccupied(double wx, double wy, bool occupied = true);
    void setCellOccupied(int gx, int gy, bool occupied = true);

    // Add rectangular obstacle in world coordinates
    void addObstacleRect(double x_center, double y_center, double width, double height, double yaw = 0.0);

    // Collision check for vehicle polygon
    bool isVehicleColliding(const std::vector<Point2D>& corners) const;

    // Load pre-configured scenario
    ScenarioInfo loadScenario(ScenarioType type);

    // Get OpenCV Mat visualization of static obstacles
    cv::Mat getBaseImage() const;

    // Get raw occupancy grid vector (0 = free, 255 = occupied)
    const std::vector<uint8_t>& getGridData() const { return m_grid; }

private:
    double m_width_m;
    double m_height_m;
    double m_resolution;
    double m_origin_x;
    double m_origin_y;
    int m_cols;
    int m_rows;
    std::vector<uint8_t> m_grid; // 0 = free, 255 = obstacle
};

} // namespace ackermann_sim

#endif // OCCUPANCY_GRID_HPP
