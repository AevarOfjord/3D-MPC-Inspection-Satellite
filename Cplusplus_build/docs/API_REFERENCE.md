# API Reference

## Core Types

### StateVector
13-DOF spacecraft state.

```cpp
struct StateVector {
    Vector3 position;           // Inertial frame [m]
    Quaternion attitude;        // Body-to-inertial [w,x,y,z]
    Vector3 velocity;           // Inertial [m/s]
    Vector3 angular_velocity;   // Body frame [rad/s]
};
```

### ControlInput
Control command output.

```cpp
struct ControlInput {
    Vector3 rw_torques;                     // [Nm]
    std::vector<double> thruster_activations; // [N] continuous thrust
};
```

---

## Control

### ControlAllocator

```cpp
class ControlAllocator {
    // Construct from vehicle config
    ControlAllocator(const VehicleConfig& config);
    
    // Get 6×N B-matrix
    const Matrix6X& get_B_matrix() const;
    
    // Recompute for fault adaptation
    void update_config(const VehicleConfig& config);
    
    // Map thrust to wrench
    Vector6 map_thrust_to_wrench(const std::vector<double>& thrust);
};
```

### MPCController

```cpp
class MPCController {
    MPCController(const VehicleConfig& vehicle, const MPCConfig& mpc);
    
    // Compute optimal control
    ControlInput compute_control(
        const StateVector& current,
        const StateVector& target);
};
```

---

## FDIR

### FaultManager

```cpp
class FaultManager {
    FaultManager(const VehicleConfig& vehicle);
    
    // Inject actuator fault
    void inject_fault(const std::string& id, FaultType fault);
    
    // Clear fault
    void clear_fault(const std::string& id);
    
    // Get config with failed actuators removed
    VehicleConfig get_effective_vehicle_config() const;
    
    // Check status
    bool has_active_faults() const;
    int get_healthy_thruster_count() const;
};

enum class FaultType {
    NONE, STUCK_OFF, STUCK_ON, DEGRADED, INTERMITTENT
};
```

---

## Planning

### InspectionPlanner

```cpp
class InspectionPlanner {
    // Generate mission from mesh file
    MissionConfig plan_mission(
        const std::string& mesh_file,
        const Vector3& start_position,
        const InspectionParams& params = {});
};

struct InspectionParams {
    double standoff_distance = 3.0;  // [m]
    double zone_size = 2.0;          // [m²]
    double hold_time = 2.0;          // [s]
};
```

### MeshAnalyzer

```cpp
class MeshAnalyzer {
    bool load(const std::string& filepath);  // STL or OBJ
    
    std::vector<InspectionZone> create_zones(double zone_size);
    std::vector<Viewpoint> generate_viewpoints(zones, distance);
    std::vector<int> solve_tsp(viewpoints, start_pos);
};
```

---

## Export

### FlightCodeGenerator

```cpp
class FlightCodeGenerator {
    FlightCodeGenerator(const GeneratorOptions& opts = {});
    
    // Generate C99 code
    bool generate(const VehicleConfig& vehicle, int horizon, double dt);
    
    // List generated files
    const std::vector<std::string>& get_generated_files() const;
};

struct GeneratorOptions {
    TargetPlatform platform = GENERIC_C;  // or ARM_CORTEX_M, FPGA_HLS
    std::string output_dir = "generated";
    std::string prefix = "sat_ctrl_";
};
```

---

## I/O

### ConfigLoader

```cpp
class ConfigLoader {
    static RootConfig load(const std::string& main_yaml);
    static VehicleConfig load_vehicle_config(const std::string& path);
    static MissionConfig load_mission_config(const std::string& path);
};
```

### HILBridge

```cpp
class HILBridge {
    HILBridge(Protocol protocol, const std::string& config);
    
    bool connect();
    void disconnect();
    bool send_state(const StateVector& state);
    void poll();  // Process incoming messages
};

enum class Protocol { SERIAL, CAN, UDP };
```
