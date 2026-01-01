#ifndef COLLISION_DETECTION_HPP
#define COLLISION_DETECTION_HPP

#include "sensor_data.hpp"
#include <cstdint>

// Collision detection configuration
constexpr float COLLISION_WARNING_TTC_SEC = 3.0f;   // Warning threshold
constexpr float COLLISION_CRITICAL_TTC_SEC = 1.5f;  // Critical threshold
constexpr float MIN_SAFE_DISTANCE_M = 2.0f;         // Minimum safe distance
constexpr float COLLISION_SPEED_THRESHOLD = 0.1f;   // Ignore very slow speeds

// Collision severity levels
enum class CollisionRisk : uint8_t {
    NONE,      // No risk
    LOW,       // Potential risk, monitoring
    WARNING,   // Warning - collision possible
    CRITICAL   // Critical - imminent collision
};

// Collision detection result
struct CollisionInfo {
    CollisionRisk risk_level;
    float time_to_collision;    // Time to collision in seconds (negative if diverging)
    float distance;             // Current distance to obstacle
    bool is_approaching;        // True if closing distance
};

class CollisionDetector {
public:
    CollisionDetector();
    
    // Analyze state for collision risk
    // Assumes obstacle at origin (0,0) for simplicity
    // In real system, would use multiple obstacles from sensor fusion
    CollisionInfo detect_collision(const StateVector& state);
    
    // Check if emergency brake should be triggered
    bool should_trigger_brake(const CollisionInfo& info) const;
    
private:
    float calculate_ttc(float distance, float closing_speed) const;
    float calculate_distance(float x, float y) const;
};

#endif // COLLISION_DETECTION_HPP
