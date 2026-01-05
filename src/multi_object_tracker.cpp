#include "multi_object_tracker.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

MultiObjectTracker::MultiObjectTracker()
    : next_track_id_(1)
{
    tracks_.reserve(MAX_TRACKED_OBJECTS);
}

void MultiObjectTracker::predict(float dt) {
    for (auto& track : tracks_) {
        if (track.status != TrackStatus::LOST) {
            predict_track(track, dt);
        }
    }
}

void MultiObjectTracker::predict_track(TrackedObject& track, float dt) {
    // State transition matrix (constant velocity model)
    Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
    F(0, 2) = dt;  // x += vx * dt
    F(1, 3) = dt;  // y += vy * dt
    
    // Process noise
    Eigen::Matrix4f Q = Eigen::Matrix4f::Identity() * 0.1f;
    Q(0, 0) = 0.25f * dt * dt * dt * dt;  // Position uncertainty
    Q(1, 1) = 0.25f * dt * dt * dt * dt;
    Q(2, 2) = dt * dt;                     // Velocity uncertainty
    Q(3, 3) = dt * dt;
    
    // Predict state: x = F * x
    track.state = F * track.state;
    
    // Predict covariance: P = F * P * F^T + Q
    track.covariance = F * track.covariance * F.transpose() + Q;
}

void MultiObjectTracker::update(const SensorData& measurement, uint64_t timestamp) {
    // Associate measurement with existing tracks
    associate_measurements(measurement, timestamp);
    
    // Remove tracks that haven't been updated
    remove_dead_tracks();
}

void MultiObjectTracker::associate_measurements(const SensorData& measurement, 
                                                uint64_t timestamp) {
    float best_distance;
    int best_match_idx = find_best_match(measurement, best_distance);
    
    if (best_match_idx >= 0 && best_distance < GATING_THRESHOLD) {
        // Update existing track
        update_track(tracks_[best_match_idx], measurement);
        tracks_[best_match_idx].last_update_time = timestamp;
        tracks_[best_match_idx].hits++;
        tracks_[best_match_idx].misses = 0;
        
        // Confirm track if enough hits
        if (tracks_[best_match_idx].hits >= MIN_HITS_FOR_CONFIRMATION) {
            tracks_[best_match_idx].status = TrackStatus::CONFIRMED;
            tracks_[best_match_idx].confidence = 
                std::min(1.0f, tracks_[best_match_idx].hits / 10.0f);
        }
    } else {
        // Create new track
        if (tracks_.size() < MAX_TRACKED_OBJECTS) {
            create_new_track(measurement, timestamp);
        }
    }
    
    // Increment age and misses for all tracks
    for (auto& track : tracks_) {
        track.age++;
        if (track.last_update_time != timestamp) {
            track.misses++;
        }
    }
}

void MultiObjectTracker::update_track(TrackedObject& track, const SensorData& measurement) {
    // Measurement vector
    Eigen::Vector4f z;
    z << measurement.x, measurement.y, measurement.vx, measurement.vy;
    
    // Measurement matrix (direct observation)
    Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
    
    // Measurement noise
    Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
    R(0, 0) = 0.5f;   // Lidar position noise
    R(1, 1) = 0.5f;
    R(2, 2) = 0.3f;   // Radar velocity noise
    R(3, 3) = 0.3f;
    
    // Innovation: y = z - H * x
    Eigen::Vector4f y = z - H * track.state;
    
    // Innovation covariance: S = H * P * H^T + R
    Eigen::Matrix4f S = H * track.covariance * H.transpose() + R;
    
    // Kalman gain: K = P * H^T * S^-1
    Eigen::Matrix4f K = track.covariance * H.transpose() * S.inverse();
    
    // Update state: x = x + K * y
    track.state = track.state + K * y;
    
    // Update covariance: P = (I - K * H) * P
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    track.covariance = (I - K * H) * track.covariance;
}

float MultiObjectTracker::calculate_mahalanobis_distance(const TrackedObject& track,
                                                         const SensorData& measurement) const {
    // Measurement vector
    Eigen::Vector4f z;
    z << measurement.x, measurement.y, measurement.vx, measurement.vy;
    
    // Predicted measurement
    Eigen::Vector4f z_pred = track.state;
    
    // Innovation
    Eigen::Vector4f y = z - z_pred;
    
    // Innovation covariance (simplified)
    Eigen::Matrix4f S = track.covariance + Eigen::Matrix4f::Identity() * 0.5f;
    
    // Mahalanobis distance: sqrt(y^T * S^-1 * y)
    float distance = std::sqrt((y.transpose() * S.inverse() * y)(0, 0));
    
    return distance;
}

int MultiObjectTracker::find_best_match(const SensorData& measurement, float& distance) const {
    int best_idx = -1;
    float min_distance = std::numeric_limits<float>::max();
    
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (tracks_[i].status == TrackStatus::LOST) {
            continue;
        }
        
        float dist = calculate_mahalanobis_distance(tracks_[i], measurement);
        
        if (dist < min_distance) {
            min_distance = dist;
            best_idx = static_cast<int>(i);
        }
    }
    
    distance = min_distance;
    return best_idx;
}

void MultiObjectTracker::create_new_track(const SensorData& measurement, uint64_t timestamp) {
    TrackedObject new_track;
    new_track.id = next_track_id_++;
    new_track.status = TrackStatus::TENTATIVE;
    new_track.obj_class = ObjectClass::UNKNOWN;
    
    // Initialize state from measurement
    new_track.state << measurement.x, measurement.y, measurement.vx, measurement.vy;
    
    // Initialize covariance (high uncertainty)
    new_track.covariance = Eigen::Matrix4f::Identity() * 10.0f;
    
    new_track.last_update_time = timestamp;
    new_track.age = 0;
    new_track.hits = 1;
    new_track.misses = 0;
    new_track.confidence = 0.1f;
    
    tracks_.push_back(new_track);
}

void MultiObjectTracker::remove_dead_tracks() {
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [](const TrackedObject& track) {
                return track.misses > MAX_MISSES_BEFORE_DELETION;
            }),
        tracks_.end()
    );
}

std::vector<TrackedObject> MultiObjectTracker::get_confirmed_tracks() const {
    std::vector<TrackedObject> confirmed;
    for (const auto& track : tracks_) {
        if (track.status == TrackStatus::CONFIRMED) {
            confirmed.push_back(track);
        }
    }
    return confirmed;
}

uint32_t MultiObjectTracker::get_track_count() const {
    return static_cast<uint32_t>(tracks_.size());
}

void MultiObjectTracker::reset() {
    tracks_.clear();
    next_track_id_ = 1;
}
