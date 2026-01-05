# ADAS ECU System Requirements

## 1. Functional Requirements

### FR-1: Sensor Data Acquisition
- **FR-1.1**: System SHALL read Lidar position data (x, y) from CAN ID 0x100
- **FR-1.2**: System SHALL read Radar velocity data (vx, vy) from CAN ID 0x200
- **FR-1.3**: Sensor task SHALL execute with 10ms period (100 Hz)
- **FR-1.4**: Sensor data SHALL be timestamped with microsecond precision

### FR-2: State Estimation
- **FR-2.1**: System SHALL implement Kalman filter for 2D position and velocity
- **FR-2.2**: State vector SHALL contain [x, y, vx, vy]
- **FR-2.3**: Kalman task SHALL execute with 20ms period (50 Hz)
- **FR-2.4**: Filter SHALL perform prediction and update steps each cycle

### FR-3: Safety Monitoring
- **FR-3.1**: Watchdog SHALL monitor compute task execution time
- **FR-3.2**: Watchdog SHALL execute with 5ms period (200 Hz)
- **FR-3.3**: System SHALL enforce 50ms deadline for compute task
- **FR-3.4**: System SHALL trigger emergency brake on deadline violation
- **FR-3.5**: System SHALL enter SAFE state on deadline violation

### FR-4: Real-Time Execution
- **FR-4.1**: System SHALL use FreeRTOS for task scheduling
- **FR-4.2**: Task priorities SHALL be: Watchdog(3) > Compute(2) > Sensor(1)
- **FR-4.3**: Tasks SHALL execute with deterministic periods
- **FR-4.4**: No task SHALL use dynamic memory allocation

---

## 2. Non-Functional Requirements

### NFR-1: Performance
- **NFR-1.1**: Sensor task WCET SHALL be < 5ms
- **NFR-1.2**: Compute task WCET SHALL be < 50ms
- **NFR-1.3**: Watchdog task WCET SHALL be < 1ms
- **NFR-1.4**: Queue insertion SHALL be O(1) bounded time

### NFR-2: Reliability
- **NFR-2.1**: System SHALL operate continuously without crashes
- **NFR-2.2**: No task SHALL starve due to priority inversion
- **NFR-2.3**: Queue overflow SHALL not cause system failure
- **NFR-2.4**: CAN errors SHALL not crash sensor task

### NFR-3: Safety
- **NFR-3.1**: System SHALL comply with ASIL-B guidelines
- **NFR-3.2**: Watchdog SHALL be independent from compute task
- **NFR-3.3**: Emergency brake SHALL activate within 10ms of violation
- **NFR-3.4**: System state SHALL be atomic (thread-safe)

### NFR-4: Maintainability
- **NFR-4.1**: Code SHALL follow MISRA C++ guidelines
- **NFR-4.2**: All timing parameters SHALL be configurable
- **NFR-4.3**: Static analysis SHALL produce zero critical warnings
- **NFR-4.4**: Code SHALL not use exceptions or RTTI

---

## 3. Constraints

### C-1: Platform
- **C-1.1**: System SHALL run on Linux POSIX (for simulation)
- **C-1.2**: System SHALL use FreeRTOS POSIX port
- **C-1.3**: System SHALL use SocketCAN for virtual CAN bus
- **C-1.4**: System SHALL be container-deployable (Docker)

### C-2: Language & Tools
- **C-2.1**: Implementation language SHALL be C++17
- **C-2.2**: Build system SHALL be CMake 3.16+
- **C-2.3**: Math library SHALL be Eigen 3.3+
- **C-2.4**: Compiler flags: `-fno-exceptions -fno-rtti`

### C-3: Memory
- **C-3.1**: NO heap allocation (`new`/`delete`)
- **C-3.2**: All data structures SHALL be fixed-size
- **C-3.3**: Stack sizes: Sensor(2KB), Compute(4KB), Watchdog(1KB)
- **C-3.4**: Queue depth SHALL be 10 elements

---

## 4. Interface Requirements

### IR-1: CAN Bus Protocol
- **IR-1.1**: Lidar frame format: `[x(4 bytes)] [y(4 bytes)]` (little-endian floats)
- **IR-1.2**: Radar frame format: `[vx(4 bytes)] [vy(4 bytes)]` (little-endian floats)
- **IR-1.3**: Brake command format: `[0xFF]` on CAN ID 0x300
- **IR-1.4**: CAN interface SHALL be `vcan0`

### IR-2: Inter-Task Communication
- **IR-2.1**: Sensor → Compute: FreeRTOS queue
- **IR-2.2**: Compute → Watchdog: Shared timing metrics
- **IR-2.3**: Watchdog → All: Atomic system state
- **IR-2.4**: No direct task-to-task calls

---

## 5. Data Requirements

### DR-1: Sensor Data Structure
```cpp
struct SensorData {
    float x, y;           // Lidar position (meters)
    float vx, vy;         // Radar velocity (m/s)
    uint64_t timestamp;   // Microseconds
};
```

### DR-2: State Vector Structure
```cpp
struct StateVector {
    float x, y;           // Position estimate (meters)
    float vx, vy;         // Velocity estimate (m/s)
    uint64_t timestamp;   // Microseconds
};
```

### DR-3: System State Enumeration
```cpp
enum class SystemState {
    INIT,      // Initializing
    RUNNING,   // Normal operation
    SAFE,      // Safe state (deadline violated)
    FAULT      // Unrecoverable fault
};
```

---

## 6. Quality Requirements

### QR-1: Testing
- **QR-1.1**: Kalman filter SHALL have unit tests
- **QR-1.2**: CAN parsing SHALL be validated with known inputs
- **QR-1.3**: Deadline violation SHALL be tested with artificial delay
- **QR-1.4**: Test coverage SHALL be > 80%

### QR-2: Documentation
- **QR-2.1**: All functions SHALL have doxygen comments
- **QR-2.2**: README SHALL contain build instructions
- **QR-2.3**: Architecture SHALL be documented with diagrams
- **QR-2.4**: Configuration parameters SHALL be documented

---

## 7. Acceptance Criteria

### AC-1: System Boots Successfully
- FreeRTOS scheduler starts
- All 3 tasks created
- CAN interface initialized
- Sensor queue created

### AC-2: Sensor Data Flows Correctly
- Sensor task reads CAN frames
- Data is queued without loss
- Compute task receives data
- Kalman filter produces estimates

### AC-3: Timing is Enforced
- Sensor runs at 10ms
- Compute runs at 20ms
- Watchdog runs at 5ms
- No deadline violations in normal operation

### AC-4: Safety Mechanism Works
- Artificial 60ms delay triggers violation
- Emergency brake command sent
- System enters SAFE state
- Violation is logged

### AC-5: Code Quality
- Zero cppcheck critical warnings
- No exceptions used
- No dynamic memory
- MISRA compliant (where applicable)

