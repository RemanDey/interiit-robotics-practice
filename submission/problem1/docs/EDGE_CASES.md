# Edge Cases

- Impossible deadline / overweight: rejected up front with a reason.
- Pad gridlock: free pad reserved, else queued (SoC < 15% jumps the queue and can preempt a reservation).
- Fleet saturation: no eligible drone → request stays Queued, retried every tick (earliest deadline first).
- Competing requests: cheaper cost (deadline + energy + workload) wins the scarce drone.
- Pad loss: `release_reservation()` frees the pad for the next request.
- Mid-flight reassignment: clear old request + route, install new route.
- Bad telemetry: rejected on SoC outside 0–100, jumps > 18%, positions beyond ±25 km, or implied speed > 28 m/s.
- Wind (`speed_factor` < 1): ETAs grow, safety checks get stricter automatically.
- Link loss: airborne drone with no heartbeat for 20 s → `Lost`; client holds, then returns home.
