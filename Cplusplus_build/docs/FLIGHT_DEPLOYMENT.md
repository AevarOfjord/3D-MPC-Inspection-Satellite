# Flight Deployment Guide

Deploy the control kernel to embedded flight computers.

## Code Generation

Generate standalone C99 code:

```cpp
#include "export/FlightCodeGenerator.h"

FlightCodeGenerator gen;
gen.generate(vehicle_config, 10, 0.05);
```

Output files in `generated/`:
```
sat_ctrl_types.h      - Type definitions
sat_ctrl_allocator.c  - B-matrix and wrench computation
sat_ctrl_controller.c - Simplified PD controller
sat_ctrl_interface.h  - Main API
Makefile              - Build script
```

## Target Platforms

```cpp
GeneratorOptions opts;
opts.platform = TargetPlatform::ARM_CORTEX_M;  // ARM Cortex-M4
opts.platform = TargetPlatform::FPGA_HLS;      // High-Level Synthesis
opts.platform = TargetPlatform::GENERIC_C;     // Standard C99
```

## Building for ARM

```bash
cd generated
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -c *.c
arm-none-eabi-ar rcs libsat_ctrl.a *.o
```

## Integration Example

```c
#include "sat_ctrl_interface.h"

void control_loop(void) {
    sat_ctrl_state_t current, target;
    sat_ctrl_control_t output;
    
    // Get sensor data
    read_imu(&current.quat, &current.omega);
    read_gps(&current.pos, &current.vel);
    
    // Compute control
    sat_ctrl_step(&current, &target, &output);
    
    // Apply to actuators
    set_rw_torques(output.rw_torque);
    set_thrusters(output.thrust);
}
```

## HIL Testing

Connect to flight hardware over serial:

```cpp
HILBridge hil(Protocol::SERIAL, "/dev/ttyUSB0");
hil.connect();

while (running) {
    // Send state from simulation
    hil.send_state(physics->get_state());
    
    // Receive control from flight computer
    hil.poll();
}
```

## Telemetry Monitoring

Stream data to ground station:

```cpp
TelemetryServer telem(5555, 5556);
telem.start();

// In control loop
auto pkt = telem.build_packet(time, state, control, ...);
telem.publish(pkt);
```

Subscribe with Python:
```python
import zmq
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect("tcp://localhost:5555")
sock.subscribe("")
while True:
    data = json.loads(sock.recv_string())
    print(data["pos"], data["err"])
```
