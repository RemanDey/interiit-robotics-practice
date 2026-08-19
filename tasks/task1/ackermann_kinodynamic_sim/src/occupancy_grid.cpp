#include "occupancy_grid.hpp"
#include <cmath>
#include <algorithm>

namespace ackermann_sim {

OccupancyGrid::OccupancyGrid(double width_m, double height_m, double resolution, 
                             double origin_x, double origin_y)
    : m_width_m(width_m), m_height_m(height_m), m_resolution(resolution),
      m_origin_x(origin_x), m_origin_y(origin_y) {
    m_cols = static_cast<int>(std::ceil(m_width_m / m_resolution));
    m_rows = static_cast<int>(std::ceil(m_height_m / m_resolution));
    m_grid.assign(m_cols * m_rows, 0); // 0 = free
}

bool OccupancyGrid::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - m_origin_x) / m_resolution));
    gy = static_cast<int>(std::floor((wy - m_origin_y) / m_resolution));
    return (gx >= 0 && gx < m_cols && gy >= 0 && gy < m_rows);
}

void OccupancyGrid::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = m_origin_x + (gx + 0.5) * m_resolution;
    wy = m_origin_y + (gy + 0.5) * m_resolution;
}

bool OccupancyGrid::isOccupied(double wx, double wy) const {
    int gx, gy;
    if (!worldToGrid(wx, wy, gx, gy)) {
        return true; // Out of bounds is treated as occupied
    }
    return isCellOccupied(gx, gy);
}

bool OccupancyGrid::isCellOccupied(int gx, int gy) const {
    if (gx < 0 || gx >= m_cols || gy < 0 || gy >= m_rows) {
        return true;
    }
    return m_grid[gy * m_cols + gx] > 127;
}

void OccupancyGrid::setOccupied(double wx, double wy, bool occupied) {
    int gx, gy;
    if (worldToGrid(wx, wy, gx, gy)) {
        setCellOccupied(gx, gy, occupied);
    }
}

void OccupancyGrid::setCellOccupied(int gx, int gy, bool occupied) {
    if (gx >= 0 && gx < m_cols && gy >= 0 && gy < m_rows) {
        m_grid[gy * m_cols + gx] = occupied ? 255 : 0;
    }
}

void OccupancyGrid::addObstacleRect(double x_center, double y_center, double width, double height, double yaw) {
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);

    double half_w = width / 2.0;
    double half_h = height / 2.0;

    double max_r = std::sqrt(half_w * half_w + half_h * half_h);
    double min_wx = x_center - max_r;
    double max_wx = x_center + max_r;
    double min_wy = y_center - max_r;
    double max_wy = y_center + max_r;

    int min_gx, min_gy, max_gx, max_gy;
    worldToGrid(min_wx, min_wy, min_gx, min_gy);
    worldToGrid(max_wx, max_wy, max_gx, max_gy);

    min_gx = std::clamp(min_gx, 0, m_cols - 1);
    max_gx = std::clamp(max_gx, 0, m_cols - 1);
    min_gy = std::clamp(min_gy, 0, m_rows - 1);
    max_gy = std::clamp(max_gy, 0, m_rows - 1);

    for (int gy = min_gy; gy <= max_gy; ++gy) {
        for (int gx = min_gx; gx <= max_gx; ++gx) {
            double wx, wy;
            gridToWorld(gx, gy, wx, wy);

            double dx = wx - x_center;
            double dy = wy - y_center;
            double lx =  dx * cos_y + dy * sin_y;
            double ly = -dx * sin_y + dy * cos_y;

            if (std::abs(lx) <= half_w && std::abs(ly) <= half_h) {
                m_grid[gy * m_cols + gx] = 255;
            }
        }
    }
}

bool OccupancyGrid::isVehicleColliding(const std::vector<Point2D>& corners) const {
    if (corners.size() < 4) return true;

    for (const auto& pt : corners) {
        if (isOccupied(pt.x, pt.y)) return true;
    }

    double step_size = m_resolution / 2.0;
    for (size_t i = 0; i < corners.size(); ++i) {
        Point2D p1 = corners[i];
        Point2D p2 = corners[(i + 1) % corners.size()];

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        int steps = static_cast<int>(std::ceil(dist / step_size));

        for (int s = 1; s < steps; ++s) {
            double ratio = static_cast<double>(s) / steps;
            double px = p1.x + ratio * dx;
            double py = p1.y + ratio * dy;
            if (isOccupied(px, py)) return true;
        }
    }

    double cx = (corners[0].x + corners[2].x) / 2.0;
    double cy = (corners[0].y + corners[2].y) / 2.0;
    if (isOccupied(cx, cy)) return true;

    return false;
}

ScenarioInfo OccupancyGrid::loadScenario(ScenarioType type) {
    std::fill(m_grid.begin(), m_grid.end(), 0);
    ScenarioInfo info;

    // Boundary walls around perimeter of 100x100m world
    for (int gx = 0; gx < m_cols; ++gx) {
        setCellOccupied(gx, 0, true);
        setCellOccupied(gx, m_rows - 1, true);
    }
    for (int gy = 0; gy < m_rows; ++gy) {
        setCellOccupied(0, gy, true);
        setCellOccupied(m_cols - 1, gy, true);
    }

    if (type == ScenarioType::PARALLEL_PARKING_STREET) {
        info.name = "Scenario 0: Street Parallel Parking";
        
        info.start_pose.x = -16.0;
        info.start_pose.y = 6.0;
        info.start_pose.yaw = 0.0;
        info.start_pose.v = 0.0;
        info.start_pose.delta = 0.0;

        info.goal_pose.x = 2.0;
        info.goal_pose.y = -1.5;
        info.goal_pose.yaw = 0.0;
        info.goal_pose.v = 0.0;
        info.goal_pose.delta = 0.0;

        // Sidewalk Curb
        addObstacleRect(0.0, -27.0, 100.0, 47.0, 0.0);

        // Parked Vehicles creating tight parking slot
        addObstacleRect(13.0, -1.5, 4.8, 2.2, 0.0);  // Front Vehicle
        addObstacleRect(-8.0, -1.5, 4.8, 2.2, 0.0);  // Rear Vehicle

        // Road lane divider / opposite traffic line
        addObstacleRect(0.0, 33.0, 100.0, 34.0, 0.0);

        // Extra bollards/barriers
        addObstacleRect(-3.5, -3.8, 0.6, 0.6, 0.0);
    }
    else if (type == ScenarioType::PARALLEL_PARKING_LOT) {
        info.name = "Scenario 1: Parking Lot Bay";

        info.start_pose.x = -25.0;
        info.start_pose.y = 12.0;
        info.start_pose.yaw = 0.0;

        info.goal_pose.x = 0.0;
        info.goal_pose.y = -8.0;
        info.goal_pose.yaw = M_PI / 2.0; // 90 degree bay parking

        // Parking rows (Top row of parked cars)
        for (double px = -35.0; px <= 35.0; px += 6.5) {
            addObstacleRect(px, 20.0, 2.2, 4.8, 0.0);
        }

        // Bottom row of parked cars with target slot at x = 0.0
        for (double px = -35.0; px <= 35.0; px += 6.5) {
            if (std::abs(px) < 3.0) continue; // Target parking bay slot
            addObstacleRect(px, -8.0, 2.2, 4.8, 0.0);
        }

        // Perimeter barriers
        addObstacleRect(0.0, -30.0, 100.0, 30.0, 0.0);
        addObstacleRect(0.0, 35.0, 100.0, 20.0, 0.0);
    }
    else if (type == ScenarioType::SLALOM_COURSE) {
        info.name = "Scenario 2: Slalom Track";

        info.start_pose.x = -40.0;
        info.start_pose.y = -40.0;
        info.start_pose.yaw = 0.0;

        info.goal_pose.x = 40.0;
        info.goal_pose.y = 40.0;
        info.goal_pose.yaw = 0.0;

        // Slalom barriers across 100x100m map
        addObstacleRect(-20.0, -15.0, 5.0, 50.0, 0.0);
        addObstacleRect(10.0, 15.0, 5.0, 50.0, 0.0);
        addObstacleRect(-30.0, 25.0, 25.0, 5.0, 0.0);
        addObstacleRect(30.0, -25.0, 25.0, 5.0, 0.0);
    }
    else { // MULTI_GOAL_WAYPOINTS
        info.name = "Scenario 3: Multi-Goal Navigation";

        info.start_pose.x = -40.0;
        info.start_pose.y = -40.0;
        info.start_pose.yaw = 0.0;

        info.goal_pose.x = 40.0;
        info.goal_pose.y = 40.0;
        info.goal_pose.yaw = 0.0;

        // Obstacles creating rooms and corridors
        addObstacleRect(-10.0, 0.0, 40.0, 3.0, 0.0);
        addObstacleRect(15.0, 20.0, 3.0, 40.0, 0.0);
        addObstacleRect(-20.0, -20.0, 3.0, 35.0, 0.0);
        addObstacleRect(10.0, -25.0, 30.0, 3.0, 0.0);

        // Sequential Waypoint targets
        info.waypoints = {
            {-30.0,  20.0, false},
            {  0.0,  35.0, false},
            { 30.0,  25.0, false},
            { 35.0, -15.0, false},
            { 10.0, -35.0, false},
            {-25.0, -20.0, false},
            {  0.0,   0.0, false},
            { 40.0,  40.0, false}
        };
    }

    return info;
}

cv::Mat OccupancyGrid::getBaseImage() const {
    cv::Mat img(m_rows, m_cols, CV_8UC3, cv::Scalar(40, 40, 40));

    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            if (isCellOccupied(c, r)) {
                img.at<cv::Vec3b>(r, c) = cv::Vec3b(90, 90, 110);
            }
        }
    }
    return img;
}

} // namespace ackermann_sim
