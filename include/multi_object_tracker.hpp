#ifndef MULTI_OBJECT_TRACKER_HPP
#define MULTI_OBJECT_TRACKER_HPP

#include "sensor_data.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cstdint>

// Maximum number of tracked objects
constexpr uint32_t MAX_TRACKED_OBJECTS = 10;

// Track status
enum class TrackStatus : uint8_t {
    TENTATIVE,   // New track, not confirmed
    CONFIRMED,   // Established track
    LOST         // Track lost, will be deleted
};

// Object classification
enum class ObjectClass : uint8_t {
    UNKNOWN = 0,
    VEHICLE = 1,
    PEDESTRIAN = 2,
    CYCLIST = 3,
    STATIC_OBSTACLE = 4
};

// Single tracked object
struct TrackedObject {
    uint32_t id;                    // Unique track ID
    TrackStatus status;             // Track status
    ObjectClass obj_class;          // Object classification
    
    Eigen::Vector4f state;          // [x, y, vx, vy]
    Eigen::Matrix4f covariance;     // State covariance
    
    uint64_t last_update_time;      // Last measurement timestamp
    uint32_t age;                   // Frames since creation
    uint32_t hits;                  // Number of successful associations
    uint32_t misses;                // Consecutive missed detections
    
    float confidence;               // Track confidence [0,1]
};

// Multi-object tracker using Extended Kalman Filter
class MultiObjectTracker {
public:
    MultiObjectTracker();
    
    // Update with new sensor measurements
    void update(const SensorData& measurement, uint64_t timestamp);
    
    // Predict all tracks forward in time
    void predict(float dt);
    
    // Get all confirmed tracks
    std::vector<TrackedObject> get_confirmed_tracks() const;
    
    // Get number of active tracks
    uint32_t get_track_count() const;
    
    // Clear all tracks
    void reset();
    
private:
    std::vector<TrackedObject> tracks_;
    uint32_t next_track_id_;
    
    // Track management
    void create_new_track(const SensorData& measurement, uint64_t timestamp);
    void associate_measurements(const SensorData& measurement, uint64_t timestamp);
    void remove_dead_tracks();
    
    // Extended Kalman Filter operations
    void predict_track(TrackedObject& track, float dt);
    void update_track(TrackedObject& track, const SensorData& measurement);
    
    // Data association
    float calculate_mahalanobis_distance(const TrackedObject& track, 
                                         const SensorData& measurement) const;
    int find_best_match(const SensorData& measurement, float& distance) const;
    
    // Track confirmation thresholds
    static constexpr uint32_t MIN_HITS_FOR_CONFIRMATION = 3;
    static constexpr uint32_t MAX_MISSES_BEFORE_DELETION = 5;
    static constexpr float GATING_THRESHOLD = 9.21f;  // Chi-squared 95% for 4-DOF
};

#endif // MULTI_OBJECT_TRACKER_HPP
