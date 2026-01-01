#include "autosar_interfaces.hpp"

namespace autosar {

ServiceRegistry& ServiceRegistry::instance() {
    static ServiceRegistry registry;
    return registry;
}

void ServiceRegistry::register_sensor_fusion(ISensorFusionService* service) {
    sensor_fusion_service_ = service;
}

void ServiceRegistry::register_object_detection(IObjectDetectionService* service) {
    object_detection_service_ = service;
}

void ServiceRegistry::register_path_planning(IPathPlanningService* service) {
    path_planning_service_ = service;
}

void ServiceRegistry::register_vehicle_control(IVehicleControlService* service) {
    vehicle_control_service_ = service;
}

ISensorFusionService* ServiceRegistry::find_sensor_fusion() {
    return sensor_fusion_service_;
}

IObjectDetectionService* ServiceRegistry::find_object_detection() {
    return object_detection_service_;
}

IPathPlanningService* ServiceRegistry::find_path_planning() {
    return path_planning_service_;
}

IVehicleControlService* ServiceRegistry::find_vehicle_control() {
    return vehicle_control_service_;
}

} // namespace autosar
