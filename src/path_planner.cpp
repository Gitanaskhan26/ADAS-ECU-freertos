#include "path_planner.hpp"
#include <cmath>
#include <algorithm>

PathPlanner::PathPlanner() {
}

Eigen::Vector2f PathPlanner::predict_object_position(const TrackedObject& obj, float dt) const {
    Eigen::Vector2f future_pos;
    future_pos(0) = obj.state(0) + obj.state(2) * dt;  // x + vx*dt
    future_pos(1) = obj.state(1) + obj.state(3) * dt;  // y + vy*dt
    return future_pos;
}

std::vector<Trajectory> PathPlanner::generate_candidates(const StateVector& ego_state) {
    std::vector<Trajectory> candidates;
    
    // Candidate maneuvers: straight, left, right, brake
    struct ManeuverParams {
        float lateral_accel;  // m/s^2
        float longitudinal_accel;  // m/s^2
    };
    
    std::vector<ManeuverParams> maneuvers = {
        {0.0f, 0.0f},      // Continue straight
        {-2.0f, -1.0f},    // Brake + left
        {2.0f, -1.0f},     // Brake + right
        {0.0f, -3.0f},     // Hard brake
    };
    
    for (const auto& maneuver : maneuvers) {
        Trajectory traj;
        traj.waypoints.reserve(TRAJECTORY_POINTS);
        
        float x = ego_state.x;
        float y = ego_state.y;
        float vx = ego_state.vx;
        float vy = ego_state.vy;
        
        for (uint32_t i = 0; i < TRAJECTORY_POINTS; ++i) {
            float t = i * TRAJECTORY_TIMESTEP;
            
            // Apply accelerations
            vx += maneuver.longitudinal_accel * TRAJECTORY_TIMESTEP;
            vy += maneuver.lateral_accel * TRAJECTORY_TIMESTEP;
            
            // Limit velocities
            vx = std::max(0.0f, std::min(30.0f, vx));
            vy = std::max(-10.0f, std::min(10.0f, vy));
            
            // Update position
            x += vx * TRAJECTORY_TIMESTEP;
            y += vy * TRAJECTORY_TIMESTEP;
            
            Waypoint wp;
            wp.x = x;
            wp.y = y;
            wp.vx = vx;
            wp.vy = vy;
            wp.timestamp = t;
            
            traj.waypoints.push_back(wp);
        }
        
        candidates.push_back(traj);
    }
    
    return candidates;
}

bool PathPlanner::check_waypoint_collision(const Waypoint& wp,
                                          const TrackedObject& obj,
                                          float time_offset) const {
    // Predict object position at this time
    Eigen::Vector2f obj_pos = predict_object_position(obj, time_offset);
    
    // Calculate distance
    float dx = wp.x - obj_pos(0);
    float dy = wp.y - obj_pos(1);
    float distance = std::sqrt(dx*dx + dy*dy);
    
    // Check if too close
    return distance < SAFETY_MARGIN_M;
}

bool PathPlanner::is_collision_free(const Trajectory& trajectory,
                                   const std::vector<TrackedObject>& obstacles) const {
    for (const auto& wp : trajectory.waypoints) {
        for (const auto& obj : obstacles) {
            if (check_waypoint_collision(wp, obj, wp.timestamp)) {
                return false;
            }
        }
    }
    return true;
}

float PathPlanner::calculate_cost(const Trajectory& traj,
                                 const std::vector<TrackedObject>& obstacles) const {
    float cost = 0.0f;
    
    // Penalize deviation from straight path
    for (const auto& wp : traj.waypoints) {
        cost += std::abs(wp.y) * 0.1f;  // Lateral deviation
    }
    
    // Penalize deceleration
    if (!traj.waypoints.empty()) {
        float final_speed = std::sqrt(
            traj.waypoints.back().vx * traj.waypoints.back().vx +
            traj.waypoints.back().vy * traj.waypoints.back().vy
        );
        cost += (30.0f - final_speed) * 0.5f;  // Prefer maintaining speed
    }
    
    // Heavily penalize collisions
    if (!is_collision_free(traj, obstacles)) {
        cost += 1000.0f;
    }
    
    // Penalize proximity to obstacles
    for (const auto& wp : traj.waypoints) {
        for (const auto& obj : obstacles) {
            Eigen::Vector2f obj_pos = predict_object_position(obj, wp.timestamp);
            float dx = wp.x - obj_pos(0);
            float dy = wp.y - obj_pos(1);
            float distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < SAFETY_MARGIN_M * 2.0f) {
                cost += 10.0f / (distance + 0.1f);  // Inverse distance cost
            }
        }
    }
    
    return cost;
}

Trajectory PathPlanner::plan_trajectory(const StateVector& ego_state,
                                       const std::vector<TrackedObject>& obstacles) {
    // Generate candidate trajectories
    auto candidates = generate_candidates(ego_state);
    
    // Evaluate each candidate
    Trajectory best_trajectory;
    float best_cost = std::numeric_limits<float>::max();
    
    for (auto& traj : candidates) {
        traj.cost = calculate_cost(traj, obstacles);
        traj.is_safe = is_collision_free(traj, obstacles);
        
        if (traj.cost < best_cost) {
            best_cost = traj.cost;
            best_trajectory = traj;
        }
    }
    
    return best_trajectory;
}

Maneuver PathPlanner::get_recommended_maneuver(const Trajectory& trajectory,
                                               const std::vector<TrackedObject>& obstacles) {
    if (!trajectory.is_safe) {
        return Maneuver::EMERGENCY_STOP;
    }
    
    // Check if any obstacle is very close
    for (const auto& obj : obstacles) {
        float distance = std::sqrt(obj.state(0)*obj.state(0) + obj.state(1)*obj.state(1));
        if (distance < 2.0f) {
            return Maneuver::BRAKE;
        }
    }
    
    // Analyze trajectory for lateral motion
    if (!trajectory.waypoints.empty()) {
        float avg_lateral = 0.0f;
        for (const auto& wp : trajectory.waypoints) {
            avg_lateral += wp.y;
        }
        avg_lateral /= trajectory.waypoints.size();
        
        if (avg_lateral < -0.5f) {
            return Maneuver::SWERVE_LEFT;
        } else if (avg_lateral > 0.5f) {
            return Maneuver::SWERVE_RIGHT;
        }
    }
    
    return Maneuver::CONTINUE;
}
