#include "vehicle_model.hpp"

namespace ackermann_sim {

VehicleModel::VehicleModel(const VehicleParams& params) : m_params(params) {}

VehicleState VehicleModel::step(const VehicleState& current_state, double target_v, double target_delta, double dt) const {
    VehicleState next_state = current_state;

    // Clamp input commands
    target_v = std::clamp(target_v, m_params.min_speed, m_params.max_speed);
    target_delta = std::clamp(target_delta, -m_params.max_steer, m_params.max_steer);

    // Apply steering rate limit
    double delta_err = target_delta - current_state.delta;
    double max_delta_change = m_params.max_steer_rate * dt;
    double actual_delta_change = std::clamp(delta_err, -max_delta_change, max_delta_change);
    next_state.delta = current_state.delta + actual_delta_change;

    // Speed dynamics (instantaneous or smooth)
    next_state.v = target_v;

    // Kinematic Bicycle Model equations around rear axle center
    next_state.x += next_state.v * std::cos(current_state.yaw) * dt;
    next_state.y += next_state.v * std::sin(current_state.yaw) * dt;
    next_state.yaw += (next_state.v / m_params.wheelbase) * std::tan(next_state.delta) * dt;

    // Normalize yaw to [-PI, PI]
    while (next_state.yaw > M_PI) next_state.yaw -= 2.0 * M_PI;
    while (next_state.yaw < -M_PI) next_state.yaw += 2.0 * M_PI;

    return next_state;
}

std::vector<Point2D> VehicleModel::getBoundingBoxCorners(const VehicleState& state) const {
    // Relative coordinates w.r.t rear axle center:
    // Rear bumper: -rear_overhang
    // Front bumper: wheelbase + front_overhang
    // Left side: +width/2
    // Right side: -width/2

    double x_rear = -m_params.rear_overhang;
    double x_front = m_params.wheelbase + m_params.front_overhang;
    double y_left = m_params.width / 2.0;
    double y_right = -m_params.width / 2.0;

    // Unrotated corner points
    Point2D local_corners[4] = {
        {x_front, y_left},   // Front-Left
        {x_front, y_right},  // Front-Right
        {x_rear,  y_right},  // Rear-Right
        {x_rear,  y_left}    // Rear-Left
    };

    std::vector<Point2D> world_corners(4);
    double cos_yaw = std::cos(state.yaw);
    double sin_yaw = std::sin(state.yaw);

    for (int i = 0; i < 4; ++i) {
        world_corners[i].x = state.x + local_corners[i].x * cos_yaw - local_corners[i].y * sin_yaw;
        world_corners[i].y = state.y + local_corners[i].x * sin_yaw + local_corners[i].y * cos_yaw;
    }

    return world_corners;
}

void VehicleModel::getWheelPositions(const VehicleState& state, Point2D& front_left, Point2D& front_right, 
                                      Point2D& rear_left, Point2D& rear_right) const {
    double half_track = m_params.width / 2.0 * 0.9;
    double cos_yaw = std::cos(state.yaw);
    double sin_yaw = std::sin(state.yaw);

    // Rear left / right
    rear_left.x  = state.x - half_track * sin_yaw;
    rear_left.y  = state.y + half_track * cos_yaw;
    rear_right.x = state.x + half_track * sin_yaw;
    rear_right.y = state.y - half_track * cos_yaw;

    // Front left / right (at wheelbase offset)
    double fx = state.x + m_params.wheelbase * cos_yaw;
    double fy = state.y + m_params.wheelbase * sin_yaw;

    front_left.x  = fx - half_track * sin_yaw;
    front_left.y  = fy + half_track * cos_yaw;
    front_right.x = fx + half_track * sin_yaw;
    front_right.y = fy - half_track * cos_yaw;
}

} // namespace ackermann_sim
