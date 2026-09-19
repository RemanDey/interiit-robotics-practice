#ifndef VEHICLE_MODEL_HPP
#define VEHICLE_MODEL_HPP

#include <vector>
#include <cmath>
#include <algorithm>

namespace ackermann_sim {

struct VehicleParams {
    double length = 4.0;         // Total vehicle length (m)
    double width = 1.8;          // Total vehicle width (m)
    double wheelbase = 2.5;      // Distance between front & rear axles (m)
    double rear_overhang = 0.8;  // Distance from rear axle to rear bumper (m)
    double front_overhang = 0.7; // Distance from front axle to front bumper (m)
    double max_steer = 0.60;     // Max steering angle (~34.3 deg)
    double max_speed = 2.5;      // Max forward speed (m/s)
    double min_speed = -1.5;     // Max reverse speed (m/s)
    double max_steer_rate = 1.0; // Max steering rate (rad/s)

    double min_turning_radius() const {
        return wheelbase / std::tan(max_steer);
    }
};

struct VehicleState {
    double x = 0.0;     // World X position of rear axle center (m)
    double y = 0.0;     // World Y position of rear axle center (m)
    double yaw = 0.0;   // Orientation heading angle (rad)
    double v = 0.0;     // Current speed (m/s)
    double delta = 0.0; // Current steering angle (rad)
};

struct Point2D {
    double x;
    double y;
};

class VehicleModel {
public:
    explicit VehicleModel(const VehicleParams& params = VehicleParams());

    const VehicleParams& getParams() const { return m_params; }
    void setParams(const VehicleParams& params) { m_params = params; }

    // Kinematic integration step (Bicycle Model)
    VehicleState step(const VehicleState& current_state, double target_v, double target_delta, double dt) const;

    // Get 4 bounding box corner points in world frame
    std::vector<Point2D> getBoundingBoxCorners(const VehicleState& state) const;

    // Get front wheel center points (for visualization)
    void getWheelPositions(const VehicleState& state, Point2D& front_left, Point2D& front_right, 
                           Point2D& rear_left, Point2D& rear_right) const;

private:
    VehicleParams m_params;
};

} // namespace ackermann_sim

#endif // VEHICLE_MODEL_HPP
