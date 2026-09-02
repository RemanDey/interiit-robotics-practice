================================================================================
PROBLEM STATEMENT 01: DRONE FLEET MANAGEMENT FOR TIMED DELIVERIES
SYSTEM SPECIFICATION & PROMPT FOR AI AGENT / TECHNICAL IMPLEMENTATION
================================================================================

1. OVERVIEW & OBJECTIVE
--------------------------------------------------------------------------------
You are tasked with building a complete software system for managing an autonomous drone delivery fleet operating under strict physical, temporal, and operational constraints.

The fleet consists of 10 drones serving a continuous, dynamic stream of incoming delivery requests. Each request has a designated delivery location, package weight, and strict arrival deadline. The system must coordinate drone scheduling, route assignment, energy management, battery degradation tracking, and charging pad queuing.

The ultimate goal is to optimize deadline fulfillment while preserving fleet health, balancing workload, preventing crashes due to energy exhaustion, and handling complex edge cases seamlessly.

--------------------------------------------------------------------------------
2. SYSTEM ARCHITECTURAL COMPONENTS
--------------------------------------------------------------------------------
The implementation must consist of three decoupled core components:

A. CENTRAL SERVER (Decision Engine & Fleet Manager)
   - Maintains real-time global state of all drones, charging pads, package queues, and active deliveries.
   - Accepts continuous delivery requests via API/socket.
   - Implements the scheduling and routing algorithm to make binding assignment decisions.
   - Computes dynamic feasibility, taking into account current battery levels, capacity degradation, payload mass, and estimated flight speeds.
   - Provides full decision explainability (ability to query why a specific drone was chosen over others at any timestamp).

B. DRONE CLIENTS (Simulated Airframes)
   - 10 distinct client processes/nodes connecting to the Central Server.
   - Constantly telemetry stream: current (x, y, z) position, current payload weight, state of charge (SoC %), battery state of health (SoH / degradation cycle count), operational status (Idle, In-Flight, Charging, Holding, Recovering).
   - Executes server flight assignments, simulates physics/energy consumption, and manages local state transitions.

C. FLEET PORTAL (Live Monitoring Interface)
   - Real-time visualization dashboard displaying:
     1. Live map of drone locations and current trajectories.
     2. Status of all 10 drones (active job, payload carried, SoC %, battery degradation level).
     3. Queue of incoming, active, and completed delivery requests.
     4. Status of all 3 charging pads (Occupied by Drone ID, Time Remaining, Queue).
     5. Real-time alerts for at-risk deliveries, late arrivals, or low-battery warnings.

--------------------------------------------------------------------------------
3. BASELINE PARAMETERS & PHYSICAL CONSTRAINTS
--------------------------------------------------------------------------------
All calculations, simulations, and decision models must adhere strictly to these quantitative parameters unless explicitly modified by an extension:

Parameter                     Value / Constraint                                Notes
----------------------------- ------------------------------------------------- -------------------------------------------------------------
Initial Fleet Size            10 Drones                                         Identical initial specs unless Extension A is active.
Maximum Payload Capacity      2.5 kg                                            Hard physical limit per drone.
Package Mass Range            0.2 kg to 2.5 kg                                  Continuous distribution across incoming requests.
Base Cruise Speed             12.0 m/s                                          Speed at 0 kg payload.
Speed Reduction Model         v(m) = v_base * (1 - alpha * (m / m_max))          Speed decreases monotonically as payload mass 'm' increases.
Full Charge Endurance         ~25 minutes at full payload (2.5 kg)             Flight endurance increases when flying lighter or unladen.
Charge Time                   ~40 minutes (0% to 100% state of charge)          Linear or realistic CC-CV charging curve.
Battery Degradation Rate      -0.05% usable capacity per full cycle             Permanent capacity loss. E.g., at 400 cycles, usable capacity = 80%.
Base Stations                 1 Base Station                                    All launches and returns originate/terminate here.
Shared Charging Pads          3 Pads for 10 Drones                              Only 1 drone per pad. A charging drone is unavailable.
Delivery Deadlines            10 to 45 minutes from request timestamp           Requests arrive continuously in real-time stream.

--------------------------------------------------------------------------------
4. CRITICAL RULES & CONSTRAINTS
--------------------------------------------------------------------------------
[RULE 1 - THE GOLDEN SAFETY RULE]:
A drone MUST NEVER be assigned a job it cannot complete, including the full return trip to an available charging pad. Range calculations MUST be computed against the drone's DEGRADED CURRENT USABLE CAPACITY, not its nominal day-one capacity. Exhausting battery mid-flight constitutes a CRASH, which is a catastrophic failure.

[RULE 2 - EXPLAINABILITY REQUIREMENT]:
For every assignment decision, the server must generate a log detailing the cost/utility breakdown comparing the selected drone against all other eligible or ineligible candidate drones.

[RULE 3 - DEADLINE RIGIDITY]:
A delivery completed after its deadline is classified as a DEADLINE FAILURE, not merely a minor score reduction.

--------------------------------------------------------------------------------
5. DECISION ENGINE & OPTIMIZATION LOGIC
--------------------------------------------------------------------------------
When a new request arrives (Payload: m, Destination: D, Deadline: T_deadline), the central server must evaluate potential choices:

Option A: Assign an Idle Drone at Base.
Option B: Divert an Airborne Drone passing near destination D (if remaining payload capacity and battery range permit without compromising its existing committed delivery).
Option C: Hold Request in Queue to await a drone currently charging or returning.

The Decision Engine must optimize a multi-objective cost function balancing:
1. Deadline Satisfaction: Priority given to requests closest to deadline breach.
2. Energy Efficiency & Payload Matching: Minimizing total kWh per package-km; avoiding using heavy-capacity range on light packages if heavy packages are queued.
3. Fleet Workload Balance: Spreading flight cycles evenly across the 10 airframes to prevent premature battery wear on a subset of drones.
4. Holding Penalty vs. Opportunity Cost: Evaluating whether holding a request yields a significantly better assignment later without causing a deadline failure.

--------------------------------------------------------------------------------
6. CHARGING & PAD QUEUE MANAGEMENT
--------------------------------------------------------------------------------
With 3 pads for 10 drones, pad contention is inevitable. The system must implement robust pad management:
- Pad Reservation Protocol: How and when a returning drone reserves a pad (at departure, en-route, or upon low-battery threshold).
- Preemptive Opportunistic Charging: Deciding whether to charge an idle drone at 30% battery when pads are free vs. keeping it ready for urgent light packages.
- Reservation Revocation / Diversion: Handling scenarios where a drone holding a pad reservation is diverted or delayed.
- Emergency Charging Priorities: Queue preemption logic when multiple drones return simultaneously with critical battery levels (< 15%).

--------------------------------------------------------------------------------
7. MANDATORY EDGE CASES & FAULT SCENARIOS TO HANDLE
--------------------------------------------------------------------------------
The system MUST handle and cleanly recover from the following non-happy-path scenarios:

1. Impossible Deadline: Request arrives with a deadline shorter than the fastest possible flight time of any drone. (Server must detect up-front and reject/flag as unfulfillable rather than blindly assigning).
2. Overweight Request: Package mass exceeds max drone payload limit (2.5 kg).
3. Pad Gridlock: All 3 pads occupied, queue full, and a returning drone arrives at low SoC.
4. Fleet Saturation: Urgent high-priority request arrives while all 10 drones are fully committed on long-distance missions.
5. Resource Bottleneck Trade-off: Two competing high-priority requests arrive simultaneously, but only one available drone can make either deadline (server must select the optimal one based on fleet utility).
6. Diversion Cancel / Pad Loss: A drone that reserved a pad is diverted mid-flight to an urgent request, freeing or transferring its pad reservation.
7. Reassignment Mid-Flight: Server reassigns a delivery while Drone A is en-route; Drone A must drop off at intermediate point or return package to base cleanly.
8. Telemetry Anomaly / Sensor Failure: A drone client sends corrupt/out-of-range battery telemetry (e.g., SoC jumps from 50% to 5%) or erratic position coordinates.
9. Environmental Degradation (Wind / Slowdown): External factors reduce ground speed by 20% mid-flight, turning an on-time delivery into an at-risk delivery. Server must detect early and execute mitigation (e.g., re-routing or re-assigning).

--------------------------------------------------------------------------------
8. BONUS EXTENSIONS (ADVANCED MODULES)
--------------------------------------------------------------------------------
If core functionality is stable, implement the following modular extensions:

EXTENSION A: HETEROGENEOUS FLEET MANAGEMENT
- Redefine fleet with 3 distinct drone profiles:
  * Heavy-Lift Drone: Max payload 5.0 kg, speed 8.0 m/s, high discharge rate.
  * Fast Light Drone: Max payload 1.0 kg, speed 18.0 m/s, highly agile.
  * Aged / Legacy Drone: Max payload 2.5 kg, speed 10.0 m/s, 20%+ capacity degraded.
- Update Decision Engine to match package profiles to optimal drone types.

EXTENSION B: COMMUNICATION FAILURES & LINK DROPS
- Simulate loss of heartbeat / network drop between server and a drone mid-flight.
- Drone Autonomous Logic: Continues current command for T_timeout seconds. If unestablished, executes safe fail-soft procedure (Hold position -> Return to Base) while preserving emergency return battery reserve.
- Server Logic: Declares drone "LOST" after T_server_timeout. Handles package reassignment.
- Reconnection Protocol: When a lost drone reconnects, it MUST NOT resume its stale command. It must report position, payload, and SoC, and await fresh instructions. Handles edge case where "lost" drone arrives at base still carrying reassigned package.

EXTENSION C: MULTI-BASE INFRASTRUCTURE
- Expand topology to 3 Base Stations with 2 Charging Pads each.
- Server must solve multi-depot routing and destination base selection considering distance, pad availability, and current drone SoC.

--------------------------------------------------------------------------------
9. SIMULATION & IMPLEMENTATION PLATFORM
--------------------------------------------------------------------------------
- Preferred Simulation Frameworks: PX4 SITL
- Communication Protocol: REST API, WebSockets, gRPC, or ROS 2 topics/services between server and drone client processes.

--------------------------------------------------------------------------------
10. EXPECTED OUTPUTS & SYSTEM DELIVERABLES
--------------------------------------------------------------------------------
When fully implemented, the repository must yield:
1. Executable Server & Client System with clear startup/run scripts.
2. Fleet Portal Visualization (GUI or Web Dashboard).
3. Logging System:
   - Request Log: Timestamp, Package ID, Weight, Origin, Destination, Assigned Drone, Expected ETA, Actual ETA, Status (Success/Late/Failed).
   - Battery Log: Cycle counts, SoH %, total kWh consumed.
   - Decision Audit Trail: Mathematical explanation for every assignment decision.
4. Performance Metrics Benchmark Report:
   - On-time delivery rate (%)
   - Total energy consumption (kWh)
   - Pad utilization rate (%)
   - Mean delay per late package
   - Fleet variance in battery degradation cycles (workload balance)
================================================================================