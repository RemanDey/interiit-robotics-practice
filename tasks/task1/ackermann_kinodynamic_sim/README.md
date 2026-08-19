# Inter IIT Tech Meet: Autonomous Vehicle Kinodynamic Simulator

This repository contains a C++ simulator designed to test motion planning and control algorithms for non-holonomic mobile robots (car-like Ackermann steering vehicles).

The simulator models vehicle dynamics in a $100\text{m} \times 100\text{m}$ grid environment and provides real-time state telemetry and occupancy maps over a TCP socket interface.

---

## Task Overview

Your objective is to write an autonomous path planning and control algorithm in C++ that connects to the simulator over TCP, receives map and pose data, plans a collision-free path to the goal pose, and streams control commands ($v, \delta$) back to navigate the vehicle safely.

### Constraints & Vehicle Model
1. **Non-Holonomic Constraints**: The car cannot move sideways. Motion is governed by the Ackermann kinematic bicycle model.
2. **Steering Bounds**: Maximum steering angle $\delta_{max} = 0.60\text{ rad} \approx 34.3^\circ$.
3. **Minimum Turning Radius**: $R_{min} = L_w / \tan(\delta_{max}) \approx 3.7\text{ meters}$ (Wheelbase $L_w = 2.5\text{m}$).
4. **Collision Detection**: The vehicle body size is $4.0\text{m} \times 1.8\text{m}$. Collisions are checked against oriented bounding boxes (OBB).

---

## Available Scenarios

| Scenario ID | Name | Description |
| :---: | :--- | :--- |
| **0** | **Street Parallel Parking** | Parallel park into a tight slot between two parked vehicles along a curb. |
| **1** | **Parking Lot Bay** | Park into a perpendicular bay in a parking lot with tight aisles. |
| **2** | **Slalom Track** | Navigate through an obstacle course track. |
| **3** | **Multi-Goal Navigation** | Navigate sequentially to multiple goal waypoints. |

---

## TCP Communication Protocol

The simulator runs a TCP server on port `8091`.

### 1. Request Configuration (`Client -> Server`)
Send string `Q\n` to receive initial pose, goal pose, vehicle parameters, and grid map:
- Response format:
  - `CONFIG <start_x> <start_y> <start_yaw> <goal_x> <goal_y> <goal_yaw> <len> <width> <wheelbase> <max_steer> <max_speed> <min_speed> <map_w> <map_h> <res> <orig_x> <orig_y> <cols> <rows>`
  - `GRID <num_cells> <val_0> <val_1> ... <val_N>` (0 = Free, 1 = Obstacle)

### 2. Stream Controls (`Client -> Server`)
Send control commands at 20 Hz:
- Format: `CTRL <target_speed_m_s> <target_steering_angle_rad>\n`
- Example: `CTRL 1.2 -0.25\n`

### 3. Upload Trajectory Visualization (`Client -> Server`)
- Format: `TRAJ <x1> <y1> <yaw1> <v1>;<x2> <y2> <yaw2> <v2>;...\n`

### 4. Telemetry Stream (`Server -> Client`)
- Format: `TELEMETRY <step_count> <time_ms> <x> <y> <yaw> <v> <delta> <is_colliding> <is_goal_reached>`

---

## Building and Running

### Prerequisites
- GCC / G++ 11+ (C++17)
- CMake 3.16+
- OpenCV 4 (`libopencv-dev`)

### Build Steps
```bash
cd ackermann_kinodynamic_sim
mkdir -p build && cd build
cmake ..
make -j4
```

### Running the Simulator
Launch the simulator with a chosen port and scenario ID:
```bash
./simulator_node 8091 0   # Scenario 0: Parallel Parking
./simulator_node 8091 1   # Scenario 1: Parking Lot Bay
./simulator_node 8091 2   # Scenario 2: Slalom Track
./simulator_node 8091 3   # Scenario 3: Multi-Goal Navigation
```

### Running the Client Template
In a separate terminal, run your client code:
```bash
./candidate_template 127.0.0.1 8091
```

Candidates should edit `apps/candidate_template.cpp` to implement their path planning and control logic.
