#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "multi_object_tracker.hpp"
#include <Eigen/Dense>
#include <vector>

// Path planning parameters
constexpr float PLANNING_HORIZON_SEC = 3.0f;      // Look-ahead time
constexpr float TRAJECTORY_TIMESTEP = 0.1f;       // 100ms steps
constexpr uint32_t TRAJECTORY_POINTS = 30;        // 3 seconds / 0.1s
constexpr float SAFETY_MARGIN_M = 1.5f;           // Clearance from obstacles

// Waypoint along planned path
struct Waypoint {
    float x, y;           // Position
    float vx, vy;         // Velocity
    float timestamp;      // Time from now
};

// Planned trajectory
struct Trajectory {
    std::vector<Waypoint> waypoints;
    bool is_safe;         // True if collision-free
    float cost;           // Trajectory cost (lower is better)
};

// Maneuver commands
enum class Maneuver : uint8_t {
    CONTINUE,             // Continue current path
    BRAKE,                // Decelerate
    SWERVE_LEFT,          // Avoid left
    SWERVE_RIGHT,         // Avoid right
    EMERGENCY_STOP        // Full stop
};

// Path planner with obstacle avoidance
class PathPlanner {
public:
    PathPlanner();
    
    // Plan trajectory given ego state and obstacles
    Trajectory plan_trajectory(const StateVector& ego_state,
                               const std::vector<TrackedObject>& obstacles);
    
    // Get recommended maneuver
    Maneuver get_recommended_maneuver(const Trajectory& trajectory,
                                      const std::vector<TrackedObject>& obstacles);
    
    // Predict object future position
    Eigen::Vector2f predict_object_position(const TrackedObject& obj, float dt) const;
    
    // Check if trajectory is collision-free
    bool is_collision_free(const Trajectory& trajectory,
                           const std::vector<TrackedObject>& obstacles) const;
    
private:
    // Generate candidate trajectories
    std::vector<Trajectory> generate_candidates(const StateVector& ego_state);
    
    // Calculate trajectory cost
    float calculate_cost(const Trajectory& traj, 
                        const std::vector<TrackedObject>& obstacles) const;
    
    // Check collision for single waypoint
    bool check_waypoint_collision(const Waypoint& wp,
                                  const TrackedObject& obj,
                                  float time_offset) const;
};

#endif // PATH_PLANNER_HPP
