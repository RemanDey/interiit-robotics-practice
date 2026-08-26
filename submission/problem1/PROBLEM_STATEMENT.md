# PS01: Drone Fleet Management for Timed Deliveries

**Category:** Systems & Optimization (Hard)  
**Target Architecture:** Central Server, 10 Drone Clients, Fleet Portal  
**Repository Target:** `Team-Deimos-IIT-Mandi/Robotic-PS`

---

## 1. System Parameters & Rules
- **Fleet Size:** 10 Drones (Identical at start, degraded over time).
- **Charging Infrastructure:** 3 Charging Pads at 1 Base Station.
- **Payload Capacity:** Max 2.5 kg per drone. Package weights: 0.2 kg to 2.5 kg.
- **Cruise Speed:** 12 m/s base (drops dynamically as payload increases).
- **Endurance:** ~25 min at full charge & max payload (longer when light/empty).
- **Charge Time:** ~40 min from empty (0%) to full (100%).
- **Battery Wear:** -0.05% usable capacity per full charge cycle (permanent degradation).
- **Deadlines:** 10 to 45 minutes from dynamic request arrival.

---

## 2. Core Operational Constraints
1. **Never-Strand Rule:** A drone must NEVER be assigned a delivery it cannot complete, including the return leg to an available charging pad based on its *current degraded capacity*.
2. **Payload-Induced Consumption:** Model faster battery discharge and lower speed under heavy loads.
3. **Pad Contention:** Charging pads accommodate only 1 drone at a time. Reservations must be managed before a drone begins its low-battery return leg.
4. **Order Diverting & Queueing:** Airborne or idle drones can divert to pick up urgent packages if range, weight, and deadline constraints permit.

---

## 3. High-Priority Edge Cases to Implement
- **Impossible Orders:** Packages exceeding 2.5 kg or deadlines shorter than flight time at max speed (reject up-front).
- **Pad Over-subscription:** All 3 pads full when an airborne drone reaches critical low battery.
- **Telemetry Errors:** Invalid GPS readings or sudden battery percentage jumps reported by clients.
- **Mid-Flight Reassignment:** Reassigning a package while a drone is carrying it.
- **Simultaneous Requests:** Multiple orders arriving at the exact same millisecond competing for one optimal drone.

---

## 4. Required Deliverables
1. **GitHub Repository:** Clean code structure containing Server, Clients, and Portal.
2. **Demo Video:** Edited run showcasing package arrivals, dynamic routing, pad management, and explicit failure recovery.
3. **Algorithm Note:** Justification of dynamic cost functions, dispatch trade-offs, and breakdown limits.
4. **Edge Case Document:** List of identified edge cases, system behavior, and test scripts.
5. **Results Summary:** Metrics on on-time delivery %, total energy used, idle time, and fleet wear balance.