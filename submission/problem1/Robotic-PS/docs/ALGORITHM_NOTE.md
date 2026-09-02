# Algorithm Note

The central server evaluates every new request against every drone and writes a decision audit before any binding assignment.

## Feasibility Gate

For request `r = (destination, payload, deadline)`, a drone is eligible only when:

- payload is at or below `2.5 kg`;
- drone is in `Idle`, `Returning`, or `Holding`;
- current SoC is not below the critical threshold;
- loaded outbound energy plus empty return energy plus emergency reserve fits inside degraded usable capacity;
- expected delivery arrival is before the deadline;
- a charging pad can be reserved or queued without violating the safety reserve.

Energy uses the degraded battery capacity:

```text
usable_capacity_wh = nominal_capacity_wh * soh_pct / 100
available_energy_wh = usable_capacity_wh * soc_pct / 100
required_wh = energy(base -> destination, payload) + energy(destination -> base, 0 kg) + reserve_wh
```

Speed decreases with payload:

```text
v(m) = 12.0 * (1 - 0.35 * m / 2.5)
```

## Cost Function

Eligible drones are ranked by:

```text
cost =
  deadline_cost +
  energy_cost +
  workload_penalty +
  pad_wait_cost +
  low_soc_cost
```

- `deadline_cost`: normalizes flight time by remaining deadline slack.
- `energy_cost`: penalizes wasteful assignments.
- `workload_penalty`: discourages repeatedly using the same batteries.
- `pad_wait_cost`: accounts for return-to-charge congestion.
- `low_soc_cost`: keeps low-battery drones available for recovery instead of routine work.

## Explainability

Each `DecisionAudit` stores:

- selected drone or `None`;
- every candidate drone;
- eligibility flag;
- ETA, energy, deadline margin, pad wait, workload penalty, total cost;
- rejection or selection reason.

The server exposes the latest audits on `GET /audit`.
