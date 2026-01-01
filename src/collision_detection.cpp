#include "collision_detection.hpp"
#include <cmath>
#include <limits>

CollisionDetector::CollisionDetector() {
}

float CollisionDetector::calculate_distance(float x, float y) const {
    return std::sqrt(x * x + y * y);
}

float CollisionDetector::calculate_ttc(float distance, float closing_speed) const {
    // If not approaching, return infinity
    if (closing_speed <= COLLISION_SPEED_THRESHOLD) {
        return std::numeric_limits<float>::infinity();
    }
    
    // TTC = distance / closing_speed
    return distance / closing_speed;
}

CollisionInfo CollisionDetector::detect_collision(const StateVector& state) {
    CollisionInfo info;
    
    // Calculate distance to obstacle (assumed at origin)
    info.distance = calculate_distance(state.x, state.y);
    
    // Calculate closing speed (negative of radial velocity)
    // Radial velocity = (position dot velocity) / distance
    float radial_velocity = 0.0f;
    if (info.distance > 0.001f) {
        radial_velocity = (state.x * state.vx + state.y * state.vy) / info.distance;
    }
    
    // Closing speed is negative radial velocity (positive when approaching)
    float closing_speed = -radial_velocity;
    info.is_approaching = (closing_speed > COLLISION_SPEED_THRESHOLD);
    
    // Calculate time to collision
    if (info.is_approaching) {
        info.time_to_collision = calculate_ttc(info.distance, closing_speed);
    } else {
        info.time_to_collision = std::numeric_limits<float>::infinity();
    }
    
    // Determine risk level
    if (!info.is_approaching) {
        info.risk_level = CollisionRisk::NONE;
    } else if (info.distance < MIN_SAFE_DISTANCE_M) {
        info.risk_level = CollisionRisk::CRITICAL;
    } else if (info.time_to_collision < COLLISION_CRITICAL_TTC_SEC) {
        info.risk_level = CollisionRisk::CRITICAL;
    } else if (info.time_to_collision < COLLISION_WARNING_TTC_SEC) {
        info.risk_level = CollisionRisk::WARNING;
    } else {
        info.risk_level = CollisionRisk::LOW;
    }
    
    return info;
}

bool CollisionDetector::should_trigger_brake(const CollisionInfo& info) const {
    return (info.risk_level == CollisionRisk::CRITICAL);
}
