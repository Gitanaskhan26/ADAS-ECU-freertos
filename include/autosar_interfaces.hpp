#ifndef AUTOSAR_INTERFACES_HPP
#define AUTOSAR_INTERFACES_HPP

#include "sensor_data.hpp"
#include "multi_object_tracker.hpp"
#include "path_planner.hpp"
#include <cstdint>
#include <functional>

/**
 * AUTOSAR-Style Service-Oriented Architecture (SOA) Interfaces
 * 
 * Following AUTOSAR Adaptive Platform service interface design:
 * - Service interfaces with methods and events
 * - Proxy/Skeleton pattern for client/server communication
 * - Asynchronous callbacks for event notification
 */

namespace autosar {

// Service identifiers
constexpr uint16_t SERVICE_ID_SENSOR_FUSION = 0x1001;
constexpr uint16_t SERVICE_ID_OBJECT_DETECTION = 0x1002;
constexpr uint16_t SERVICE_ID_PATH_PLANNING = 0x1003;
constexpr uint16_t SERVICE_ID_VEHICLE_CONTROL = 0x1004;

// Service instance identifiers
constexpr uint16_t INSTANCE_ID_DEFAULT = 0x0001;

// Return codes (AUTOSAR E2E)
enum class ServiceReturnCode : uint8_t {
    OK = 0x00,
    NOT_OK = 0x01,
    PENDING = 0x02,
    TIMEOUT = 0x03,
    NOT_AVAILABLE = 0x04
};

// ============================================================================
// Sensor Fusion Service Interface
// ============================================================================

struct SensorFusionData {
    StateVector ego_state;
    uint64_t timestamp;
    uint8_t quality_indicator;  // 0-100 quality score
};

// Service event callback
using SensorFusionEventCallback = std::function<void(const SensorFusionData&)>;

class ISensorFusionService {
public:
    virtual ~ISensorFusionService() = default;
    
    // Methods
    virtual ServiceReturnCode get_ego_state(SensorFusionData& data) = 0;
    virtual ServiceReturnCode set_measurement(const SensorData& measurement) = 0;
    
    // Events
    virtual void subscribe_state_update(SensorFusionEventCallback callback) = 0;
    virtual void unsubscribe_state_update() = 0;
};

// ============================================================================
// Object Detection Service Interface
// ============================================================================

struct ObjectList {
    std::vector<TrackedObject> objects;
    uint32_t frame_id;
    uint64_t timestamp;
};

using ObjectDetectionEventCallback = std::function<void(const ObjectList&)>;

class IObjectDetectionService {
public:
    virtual ~IObjectDetectionService() = default;
    
    // Methods
    virtual ServiceReturnCode get_object_list(ObjectList& objects) = 0;
    virtual ServiceReturnCode get_object_count(uint32_t& count) = 0;
    
    // Events
    virtual void subscribe_object_detected(ObjectDetectionEventCallback callback) = 0;
    virtual void unsubscribe_object_detected() = 0;
};

// ============================================================================
// Path Planning Service Interface
// ============================================================================

struct PathPlanRequest {
    StateVector ego_state;
    ObjectList obstacles;
    float planning_horizon;
};

struct PathPlanResponse {
    std::vector<Waypoint> trajectory;
    bool is_safe;
    Maneuver recommended_maneuver;
    float confidence;
};

using PathPlanEventCallback = std::function<void(const PathPlanResponse&)>;

class IPathPlanningService {
public:
    virtual ~IPathPlanningService() = default;
    
    // Methods (synchronous)
    virtual ServiceReturnCode plan_path(const PathPlanRequest& request,
                                       PathPlanResponse& response) = 0;
    
    // Methods (asynchronous)
    virtual ServiceReturnCode plan_path_async(const PathPlanRequest& request,
                                             std::function<void(const PathPlanResponse&)> callback) = 0;
    
    // Events
    virtual void subscribe_path_updated(PathPlanEventCallback callback) = 0;
    virtual void unsubscribe_path_updated() = 0;
};

// ============================================================================
// Vehicle Control Service Interface
// ============================================================================

struct VehicleCommand {
    float target_acceleration;  // m/s^2
    float target_steering;      // radians
    uint64_t timestamp;
    bool emergency_brake;
};

struct VehicleStatus {
    float current_speed;        // m/s
    float current_acceleration; // m/s^2
    float current_steering;     // radians
    bool brake_active;
    uint64_t timestamp;
};

using VehicleStatusEventCallback = std::function<void(const VehicleStatus&)>;

class IVehicleControlService {
public:
    virtual ~IVehicleControlService() = default;
    
    // Methods
    virtual ServiceReturnCode send_command(const VehicleCommand& cmd) = 0;
    virtual ServiceReturnCode get_status(VehicleStatus& status) = 0;
    virtual ServiceReturnCode trigger_emergency_brake() = 0;
    
    // Events
    virtual void subscribe_status_changed(VehicleStatusEventCallback callback) = 0;
    virtual void unsubscribe_status_changed() = 0;
};

// ============================================================================
// Service Registry (AUTOSAR Service Discovery)
// ============================================================================

class ServiceRegistry {
public:
    static ServiceRegistry& instance();
    
    // Register/unregister services
    void register_sensor_fusion(ISensorFusionService* service);
    void register_object_detection(IObjectDetectionService* service);
    void register_path_planning(IPathPlanningService* service);
    void register_vehicle_control(IVehicleControlService* service);
    
    // Find services
    ISensorFusionService* find_sensor_fusion();
    IObjectDetectionService* find_object_detection();
    IPathPlanningService* find_path_planning();
    IVehicleControlService* find_vehicle_control();
    
private:
    ServiceRegistry() = default;
    
    ISensorFusionService* sensor_fusion_service_ = nullptr;
    IObjectDetectionService* object_detection_service_ = nullptr;
    IPathPlanningService* path_planning_service_ = nullptr;
    IVehicleControlService* vehicle_control_service_ = nullptr;
};

} // namespace autosar

#endif // AUTOSAR_INTERFACES_HPP
