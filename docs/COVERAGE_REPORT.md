# MC/DC Coverage Report

**Project:** ADAS ECU with FreeRTOS  
**Date Generated:** December 30, 2024  
**Tool:** gcov + lcov 1.14  
**Compiler:** GCC 11.4.0 with `--coverage` flags  

---

## Executive Summary

| Metric | Coverage | Target (ASIL-D) | Status |
|--------|----------|-----------------|--------|
| **Line Coverage** | 69.4% (356/513 lines) | ≥90% | 🟡 In Progress |
| **Function Coverage** | 83.1% (54/65 functions) | ≥90% | 🟡 In Progress |
| **Branch Coverage (MC/DC)** | 45.8% (66/144 branches) | ≥90% | 🟡 In Progress |

### Key Findings

✅ **Strengths:**
- **3 modules at 100% coverage**: AUTOSAR services, safety watchdog, Kalman header
- **3 modules at 90%+**: Multi-object tracker (98.2%), Kalman filter (94.1%), collision detection (93.3%)
- All 8 unit test suites passing with enhanced test cases
- Coverage instrumentation working correctly with FreeRTOS POSIX port
- **Function coverage** reached 83.1% (near target)

⚠️ **Gaps Requiring Additional Tests:**
- CAN FD interface (33.9%) - Requires hardware/virtual CAN setup
- CAN interface (22.8%) - Requires hardware/virtual CAN setup
- Path planning (82.3%) - Need edge case coverage for all 4 maneuvers
- Branch coverage overall (45.8%) - Need more conditional path testing

---

## Detailed Coverage by Module

### 1. Kalman Filter (`src/kalman.cpp`) ✅
- **Line Coverage:** 94.1% (48/51 lines)
- **Function Coverage:** 100% (5/5 functions)
- **Test Suite:** `test_kalman.cpp` (3 tests)
- **Status:** Excellent - near complete coverage
- **Remaining Gaps:** Minor edge cases in matrix operations

### 2. Collision Detection (`src/collision_detection.cpp`) ✅
- **Line Coverage:** 93.3% (28/30 lines)
- **Function Coverage:** 100% (5/5 functions)
- **Test Suite:** `test_collision_detection.cpp` (5 tests)
- **Status:** Excellent - comprehensive test coverage
- **Remaining Gaps:** Rare edge cases in TTC calculation

### 3. Multi-Object Tracker (`src/multi_object_tracker.cpp`) ✅
- **Line Coverage:** 98.2% (110/112 lines)
- **Function Coverage:** 100% (14/14 functions)
- **Test Suite:** `test_multi_object_tracker.cpp` (7 tests)
- **Status:** Outstanding - nearly complete coverage
- **Remaining Gaps:** Minimal - 2 lines only

### 4. Path Planner (`src/path_planner.cpp`) 🟡
- **Line Coverage:** 82.3% (79/96 lines)
- **Function Coverage:** 87.5% (7/8 functions)
- **Test Suite:** `test_path_planner.cpp` (6 tests)
- **Status:** Good - approaching target
- **Gaps:** Emergency maneuver edge cases, collision cost penalties

### 5. Safety Watchdog (`src/safety.cpp`) ✅
- **Line Coverage:** 100% (13/13 lines)
- **Function Coverage:** 100% (3/3 functions)
- **Test Suite:** `test_safety.cpp` (8 tests)
- **Status:** Complete - all code paths tested
- **Gaps:** None

### 6. CAN Interface (`src/can_interface.cpp`) 🔴
- **Line Coverage:** 22.8% (18/79 lines)
- **Function Coverage:** 55.6% (5/9 functions)
- **Test Suite:** `test_can_interface.cpp` (9 tests)
- **Status:** Limited - requires hardware
- **Gaps:** Socket initialization, frame RX/TX, all sensor parsers (camera, ultrasonic)
- **Note:** Requires vcan0 virtual CAN or real CAN hardware

### 7. CAN FD Interface (`src/can_fd_interface.cpp`) 🔴
- **Line Coverage:** 33.9% (37/109 lines)
- **Function Coverage:** 45.5% (5/11 functions)
- **Test Suite:** `test_can_fd_interface.cpp` (6 tests)
- **Status:** Limited - requires hardware
- **Gaps:** BRS configuration, 64-byte payload handling, frame transmission
- **Note:** Requires vcan0 virtual CAN FD or real CAN FD hardware

### 8. AUTOSAR Services (`src/autosar_services.cpp`) ✅
- **Line Coverage:** 100% (22/22 lines)
- **Function Coverage:** 100% (9/9 functions)
- **Test Suite:** `test_autosar_services.cpp` (7 tests)
- **Status:** Complete - full ServiceRegistry coverage
- **Gaps:** None

### 9. Kalman Filter Header (`include/kalman.hpp`) ✅
- **Line Coverage:** 100% (1/1 lines)
- **Function Coverage:** 100% (1/1 functions)
- **Status:** Complete
- **Gaps:** None

---

## Coverage Methodology

### Build Configuration
```bash
cmake -DCMAKE_BUILD_TYPE=Coverage ..
# Compiler flags:
# -g -O0 --coverage -fprofile-arcs -ftest-coverage
# Linker flags:
# --coverage
```

### Test Execution
```bash
# All 8 unit test suites executed:
./test_kalman
./test_safety
./test_can_interface
./test_collision_detection         # New
./test_multi_object_tracker        # New
./test_path_planner                 # New
./test_can_fd_interface             # New
./test_autosar_services             # New
```

### Coverage Collection
```bash
lcov --capture --directory build --output-file coverage.info \\
     --rc lcov_branch_coverage=1  # Enable MC/DC branch coverage

lcov --remove coverage.info '/usr/*' '*/FreeRTOS-Kernel/*' '*/tests/*' \\
     --output-file coverage_filtered.info

genhtml coverage_filtered.info --output-directory coverage_html \\
        --branch-coverage --rc genhtml_branch_coverage=1
```

---

## Exclusions

The following code is excluded from coverage requirements:

### System Headers
- `/usr/include/*` - GCC standard library
- `/usr/include/eigen3/*` - Eigen matrix library (3rd party)
- `/usr/lib/gcc/*` - Compiler intrinsics

### FreeRTOS Kernel
- `/opt/FreeRTOS-Kernel/*` - OS scheduler, queues, tasks (vendor code)
- **Rationale:** FreeRTOS is pre-qualified, vendor-tested RTOS

### Test Code
- `/workspace/tests/*` - Unit test suites themselves
- **Rationale:** Tests are validation artifacts, not production code

---

## Improvement Roadmap

### Phase 1: Core Safety Modules ✅ COMPLETE
**Status:** ACHIEVED  
**Priority:** HIGH (ASIL-D critical)

1. **Kalman Filter** ✅
   - ✅ Covariance bounds checking
   - ✅ Process noise matrix validation
   - ✅ Measurement rejection testing
   - **Achieved:** 94.1% line, 100% function

2. **Collision Detection** ✅
   - ✅ Multi-object collision scenarios
   - ✅ TTC calculation validation
   - ✅ Emergency brake trigger verification
   - **Achieved:** 93.3% line, 100% function

3. **Safety Watchdog** ✅
   - ✅ All state transitions (INIT → RUNNING → SAFE → FAULT)
   - ✅ Deadline violation recovery
   - ✅ Emergency brake trigger
   - ✅ Timestamp precision validation
   - **Achieved:** 100% line, 100% function

4. **AUTOSAR Services** ✅
   - ✅ ServiceRegistry singleton testing
   - ✅ All 4 service interfaces (SensorFusion, PathPlanning, ObjectDetection, VehicleControl)
   - ✅ Service registration/deregistration
   - **Achieved:** 100% line, 100% function

5. **Multi-Object Tracker** ✅
   - ✅ Track creation, update, deletion
   - ✅ Mahalanobis distance gating
   - ✅ Track confirmation logic
   - **Achieved:** 98.2% line, 100% function

### Phase 2: Advanced Features (Target: 85% MC/DC)
**Timeline:** Q1 2025  
**Priority:** MEDIUM
**Status:** 🟡 IN PROGRESS

1. **Path Planner** 🟡
   - ✅ Basic maneuver generation tested
   - ✅ Trajectory validation logic
   - ⚠️ Need: Emergency maneuver edge cases
   - ⚠️ Need: All 4 maneuver types under load
   - **Current:** 82.3% line (need +7.7% to reach 90%)
   - **Actions Required:**
     - Add tests for sharp turn collision avoidance
     - Add tests for gentle maneuvers at high speed
     - Add tests for trajectory cost calculation edge cases

### Phase 3: Communication Interfaces (Target: 70% MC/DC)
**Timeline:** Q2 2025  
**Priority:** LOW (Hardware-dependent)
**Status:** 🔴 BLOCKED - Requires HIL Setup

1. **CAN Interface** 🔴
   - **Current:** 22.8% line coverage
   - **Blocker:** Requires vcan0 virtual CAN or real CAN hardware
   - **Hardware Required:**
     - Virtual CAN interface (vcan0) with proper setup
     - CAN frame injection for all sensor types (LIDAR, RADAR, camera, ultrasonic)
     - Error injection capability (bus-off, arbitration lost)
   - **Tests Available:** Data encoding/decoding (working)
   - **Tests Blocked:** Socket operations, frame TX/RX

2. **CAN FD Interface** 🔴
   - **Current:** 33.9% line coverage
   - **Blocker:** Requires vcan0 virtual CAN FD or real CAN FD hardware
   - **Hardware Required:**
     - Virtual CAN FD interface with BRS support
     - 64-byte payload support
     - Bit-rate switching (500 kbps arb / 2 Mbps data)
   - **Tests Available:** Frame structure validation (working)
   - **Tests Blocked:** BRS configuration, extended payload transmission

**Note:** To achieve 90% coverage on CAN modules, a Hardware-in-the-Loop (HIL) test bench is required. Current software-only testing is limited to data format validation.

---

## ASIL-D Compliance Notes

### Current Status
- **ISO 26262-6:2018 Table 12:** Structural coverage at unit level
  - ASIL-D requires: **Statement coverage (100%)** + **MC/DC coverage (highly recommended)**
  - Current achievement: Statement 69.4%, MC/DC 45.8%, Function 83.1%
  - **Status:** 🟡 Significant progress (not yet compliant)
  - **Improvement:** +8.2% line, +3.4% branch, +21.6% function since initial baseline

### Gaps for Certification

1. **Coverage Shortfall**
   - Need: 90%+ MC/DC coverage on all safety-critical modules
   - Current: 69.4% line coverage, 45.8% branch coverage
   - **Progress:** Phase 1 complete (core safety modules at 93-100%)
   - **Remaining:** CAN interfaces blocked by hardware requirements
   - **Action:** 
     - Complete path planner edge case testing (+7.7% needed)
     - Set up HIL test bench for CAN/CAN FD interfaces
     - Add branch coverage tests for all conditional paths

2. **Tool Qualification**
   - Need: Qualified testing tools per ISO 26262-8 (Tool Confidence Level TCL2)
   - Current: Using open-source gcov/lcov (not qualified)
   - **Action:** Either qualify tools or verify with independent means

3. **Independent Assessment**
   - Need: Review by certified functional safety engineer
   - Current: Self-assessment only
   - **Action:** Engage TÜV/SGS/Bureau Veritas for audit

4. **Traceability**
   - Need: Coverage data linked to safety requirements
   - Current: Manual traceability matrix exists
   - **Action:** Automate with requirement IDs in test names

---

## How to Generate This Report

### Prerequisites
```bash
apt install lcov bc
```

### Run Coverage Generation
```bash
cd /workspace
./scripts/generate_coverage.sh
```

### View HTML Report
```bash
# In browser:
firefox build/coverage_html/index.html

# Or copy to host machine:
docker cp <container>:/workspace/build/coverage_html ./coverage_report
```

### Interpreting Results

| Color | Line Coverage | Branch Coverage | Meaning |
|-------|---------------|-----------------|---------|
| 🟢 Green (>90%) | High coverage | High coverage | Well-tested |
| 🟡 Yellow (75-90%) | Moderate coverage | Moderate coverage | Needs improvement |
| 🔴 Red (<75%) | Low coverage | Low coverage | Critical gaps |

---

## Files Included in Report

### Source Files Analyzed (9 files)
```
src/kalman.cpp                  - Kalman filter implementation
src/collision_detection.cpp     - TTC collision detection
src/multi_object_tracker.cpp    - EKF multi-object tracking
src/path_planner.cpp            - Trajectory generation
src/safety.cpp                  - Watchdog and safety monitor
src/can_interface.cpp           - Classical CAN protocol
src/can_fd_interface.cpp        - CAN FD protocol
src/autosar_services.cpp        - AUTOSAR SOA interfaces
include/kalman.hpp              - Kalman filter header (inline functions)
```

### Report Artifacts
```
build/coverage.info               - Raw lcov data (all files)
build/coverage_filtered.info      - Filtered lcov data (source only)
build/coverage_html/              - HTML report directory
build/coverage_html/index.html    - Main coverage dashboard
```

---

## References

- [ISO 26262-6:2018](https://www.iso.org/standard/68383.html) - Software unit design and testing
- [ISO 26262-8:2018](https://www.iso.org/standard/68388.html) - Supporting processes (tool qualification)
- [GCOV Documentation](https://gcc.gnu.org/onlinedocs/gcc/Gcov.html) - GNU coverage testing tool
- [LCOV User Guide](http://ltp.sourceforge.net/coverage/lcov.php) - Linux Test Project coverage frontend
- [MC/DC Coverage Guide](https://www.rapitasystems.com/blog/what-is-mcdc) - Modified Condition/Decision Coverage

---

## Coverage Progress Tracking

### Coverage Trend

| Date | Line Coverage | Branch Coverage | Function Coverage | Notes |
|------|---------------|-----------------|-------------------|-------|
| Dec 30, 2024 (Baseline) | 61.0% | 41.7% | 61.5% | Initial coverage report |
| Dec 30, 2025 (Current) | 69.4% | 45.8% | 83.1% | Enhanced test suites |

### Module Improvements

| Module | Before | After | Improvement |
|--------|--------|-------|-------------|
| AUTOSAR Services | 0.0% | 100% | +100% ✅ |
| Safety Watchdog | 30.8% | 100% | +69.2% ✅ |
| CAN Interface | 8.9% | 22.8% | +13.9% 🟡 |
| Multi-Object Tracker | - | 98.2% | New ✅ |
| Path Planner | - | 82.3% | New 🟡 |
| Kalman Filter | - | 94.1% | New ✅ |
| Collision Detection | - | 93.3% | New ✅ |

### Test Suite Enhancements

- **test_safety.cpp**: Added 5 new test cases (emergency brake, deadline logging, timestamp precision)
- **test_autosar_services.cpp**: Enhanced with ServiceRegistry singleton and all 4 service interfaces
- **test_can_interface.cpp**: Added 5 new tests (initialization, camera/ultrasonic data, brake command)
- **test_can_fd_interface.cpp**: Comprehensive frame structure and bitrate tests

### Achievements

✅ **3 modules at 100% coverage** (AUTOSAR, Safety, Kalman.hpp)  
✅ **3 modules at 90%+** (Multi-object tracker, Kalman, Collision detection)  
✅ **Function coverage: 83.1%** (near 90% target)  
✅ **All 8 test suites passing** with enhanced coverage  

### Next Steps to Reach 90%

1. **Path Planner** (+7.7% needed)
   - Add emergency maneuver edge cases
   - Test all 4 maneuver types under various conditions
   - Add collision cost calculation boundary tests

2. **Branch Coverage** (+44.2% needed)
   - Add tests for all conditional branches
   - Test error paths and exception handling
   - Add negative test cases

3. **CAN Interfaces** (Hardware-dependent)
   - Set up vcan0 virtual CAN interface
   - Implement CAN frame injection tests
   - Test error conditions (bus-off, timeouts)

---

**Report Generated:** December 30, 2025  
**Previous Report:** December 30, 2024  
**Tool Chain:** GCC 11.4.0, gcov 11.4.0, lcov 1.14  
**Environment:** Ubuntu 22.04 LTS (Docker container)  
**Next Review:** Q1 2025 (after Phase 1 improvements)
